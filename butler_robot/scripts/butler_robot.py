#!/usr/bin/env python3

import rospy
import actionlib
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import select
import sys
import signal

WAYPOINTS = {
    "kitchen": (7.675238132476807, -6.01118278503418, 1.0),
    "home": (0.0, 0.0, 1.0),
    "table1": (12.085535049438477, 4.3402838706970215, 0.6417557517388287),
    "table2": (-10.501110076904297, 5.375353813171387, 0.8235567479660424),
    "table3": (-7.50412654876709, -9.034976959228516, 0.21287361489247497),
}

CONFIRM_TIMEOUT = 30.0

IDLE = "IDLE"
DELIVERY = "DELIVERY"
WAITING_CONFIRMATION = "WAITING_CONFIRMATION"
CANCELED = "CANCELED"


class RobotButler:
    def __init__(self):
        rospy.init_node("robot_butler_service")

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        self.state = IDLE
        self.goal_location = None
        self.order_queue = []
        self.lock = threading.Lock()
        self.is_goal_cancel = False

        rospy.loginfo("Robot Butler service initialized")
        signal.signal(signal.SIGINT, self.signal_handler)

        threading.Thread(target=self.get_orders_from_user).start()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Cancellation request received. Pressing Ctrl+C")
        self.lock.acquire()
        self.state = CANCELED
        self.move_base_client.cancel_goal()
        
        if self.goal_location == "kitchen":
            rospy.loginfo("Task canceled while going to the kitchen. Returning home")
            self.return_to_home()
        elif self.goal_location and "table" in self.goal_location:
            rospy.loginfo(
                "Task canceled while going to a table. Returning to kitchen first"
            )
            self.is_goal_cancel = True
            self.return_to_kitchen()
        else:
            rospy.loginfo("Task canceled, returning home")
            self.return_to_home()

        self.lock.release()

    def get_orders_from_user(self):
        """
        Continuously requests the user for executing orders from tables
        """
        while True:
            if self.state == IDLE:
                order_input = input(
                    "Enter table numbers (comma separated, or type 'exit' to quit): "
                ).strip()
                if order_input.lower() == "exit":
                    rospy.signal_shutdown("User exited")
                    break
                self.lock.acquire()
                tables = order_input.split(",")
                valid_tables = []
                for table in tables:
                    table_name = f"table{table.strip()}"
                    if table_name in WAYPOINTS:
                        valid_tables.append(table_name)
                if valid_tables:
                    self.order_queue.extend(valid_tables)
                    rospy.loginfo(f"Orders received for: {', '.join(valid_tables)}")
                    self.lock.release()
                    threading.Thread(target=self.process_order).start()
                else:
                    rospy.loginfo("Invalid table numbers, please try again")
                    self.lock.release()

            sleep(1)

    def get_goal(self, x, y, w):
        """
        Creates a MoveBaseGoal with the specified coordinates and orientation
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w
        return goal

    def send_goal(self, x, y, w):
        """
        Sends a goal to the move_base action server and waits for the result
        """
        goal = self.get_goal(x, y, w)
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
            return True
        else:
            rospy.loginfo(f"Failed to reach goal with state: {state}")
            return False

    def process_order(self):
        """
        Processes the current order queue by moving to the kitchen, waiting for confirmation,
        and delivering to each table
        """
        self.lock.acquire()
        if self.state != IDLE:
            rospy.loginfo("Robot is busy with another task")
            self.lock.release()
            return

        self.state = DELIVERY
        self.lock.release()

        modify_thread = threading.Thread(target=self.modify_orders_while_moving)
        modify_thread.start()

        if not self.move_to_kitchen():
            rospy.loginfo("Failed to reach the kitchen, aborting task")
            self.state = IDLE
            return

        if not self.wait_for_confirmation("kitchen"):
            rospy.loginfo("No confirmation at the kitchen, returning to home")
            self.return_to_home()
            self.state = IDLE
            return

        modify_thread.join()

        while self.order_queue and self.state != CANCELED:
            current_table = self.order_queue.pop(0)
            self.goal_location = current_table

            if not self.move_to_table(current_table):
                rospy.loginfo(f"Failed to reach {current_table}, skipping")
                continue

            confirmed = self.wait_for_confirmation(current_table)
            if not confirmed:
                rospy.loginfo(
                    f"No confirmation at {current_table}, moving to the next table"
                )
                continue

        if self.state == CANCELED:
            rospy.loginfo("Order processing was canceled, returning to kitchen")
            self.return_to_kitchen()
        else:
            self.return_to_home()
        self.state = IDLE

    def modify_orders_while_moving(self):
        """
        Gets input from the user for the orders to be modified
        """
        timeout_duration = 8
        while self.state == DELIVERY:
            rospy.loginfo("Do you want to add or remove tables? (add/remove/none):")

            input_ready, _, _ = select.select([sys.stdin], [], [], timeout_duration)

            if input_ready:
                modify_input = sys.stdin.readline().strip().lower()
            else:
                rospy.loginfo(
                    f"No input received for {timeout_duration} seconds. Closing modification thread"
                )
                return

            if modify_input == "none":
                rospy.loginfo("No modifications requested")
                break

            if modify_input == "add":
                self.add_tables()
                break

            elif modify_input == "remove":
                table_input = input(
                    f"Enter the table number(s) to remove (comma separated): "
                ).strip()
                tables_to_remove = []
                for table in table_input.split(","):
                    table_name = f"table{table.strip()}"
                    if table_name in self.order_queue:
                        tables_to_remove.append(table_name)

                if not tables_to_remove:
                    rospy.loginfo(
                        "No valid tables to remove. Please enter valid tables"
                    )
                    continue

                for table in tables_to_remove:
                    if table in self.order_queue:
                        self.order_queue.remove(table)
                        rospy.loginfo(f"Removed {table} from the queue")

                if not self.order_queue:
                    rospy.loginfo(
                        "No more tables in the order queue, canceling kitchen trip and returning home"
                    )
                    self.move_base_client.cancel_goal()
                    self.return_to_home()
                    self.state = IDLE
                    return
                break
            else:
                rospy.loginfo("Invalid input. Please enter 'add', 'remove', or 'none'")

    def move_to_kitchen(self):
        """ 
        Moves the robot to the kitchen and waits for the result
        """
        self.goal_location = "kitchen"
        rospy.loginfo("Moving to kitchen..")
        x, y, w = WAYPOINTS["kitchen"]
        if not self.send_goal(x, y, w):
            rospy.loginfo("Failed to reach the kitchen, aborting task")
            return False
        return True

    def add_tables(self):
        """
        Gets input from the user to add table numbers to the order queue
        """
        table_input = input("Enter table numbers to add (comma separated): ").strip()
        tables = table_input.split(",")

        valid_tables = []
        for table in tables:
            table_name = f"table{table.strip()}"
            if table_name in WAYPOINTS and table_name not in self.order_queue:
                valid_tables.append(table_name)

        if valid_tables:
            self.order_queue.extend(valid_tables)
            rospy.loginfo(f"Added tables: {', '.join(valid_tables)}")
        else:
            rospy.loginfo("No valid tables to add or they are already in the queue")

    def move_to_table(self, table):
        """
        Moves the robot to the specified table and waits for the result
        """
        self.goal_location = table
        rospy.loginfo(f"Moving to {table}..")
        x, y, w = WAYPOINTS[table]
        return self.send_goal(x, y, w)

    def return_to_kitchen(self):
        """
        Moves the robot back to the kitchen and checks if the task was canceled
        """
        rospy.loginfo("Returning to kitchen..")
        x, y, w = WAYPOINTS["kitchen"]
        self.send_goal(x, y, w)
        rospy.logerr("self.is_goal_cancel :" + str(self.is_goal_cancel))
        # self.move_base_client.wait_for_result()
        if (
            self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED
            and self.is_goal_cancel
        ):
            self.return_to_home()
            self.is_goal_cancel = False

    def return_to_home(self):
        """
        Moves the robot back to the home position.
        """
        rospy.loginfo("Returning to home..")
        x, y, w = WAYPOINTS["home"]
        self.send_goal(x, y, w)

    def timed_input(self, prompt, timeout):
        sys.stdout.write(prompt)
        sys.stdout.flush()
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.readline().strip()
        else:
            return None

    def wait_for_confirmation(self, location):
        """
        Waits for user confirmation at a specified location within a timeout
        """
        self.goal_location = location
        rospy.loginfo(f"Waiting for confirmation at {location}..")

        start_time = rospy.Time.now()
        confirmed = False
        input_received = False

        while (rospy.Time.now() - start_time).to_sec() < CONFIRM_TIMEOUT:
            user_input = self.timed_input(f"Confirm at {location} (yes/no): ", 5)
            if user_input is None:
                rospy.loginfo("No input received, retrying..")
            else:
                user_input = user_input.strip().lower()

                if user_input == "yes":
                    confirmed = True
                    input_received = True
                    rospy.loginfo(f"Confirmation received at {location}")
                    break
                elif user_input == "no":
                    confirmed = False
                    input_received = True
                    rospy.loginfo(f"Confirmation declined at {location}")
                    break
                else:
                    rospy.loginfo("Invalid input, please type 'yes' or 'no'")

            if self.state == CANCELED:
                rospy.loginfo(f"Task canceled at {location}")
                return False

        if not input_received:
            if "table" in location:
                rospy.loginfo(f"No confirmation received at {location} within timeout")
                rospy.loginfo(
                    "Returning to kitchen and home due to lack of confirmation"
                )
                self.return_to_kitchen()
                self.return_to_home()
                return False
            else:
                rospy.loginfo(f"No confirmation received at {location} within timeout")
                rospy.loginfo("Returning to home due to lack of confirmation")
                self.return_to_home()
                return False

        return confirmed

    def ask_modify_orders(self):
        """
        Requests the user to modify orders while the robot is delivering
        """
        while self.state == DELIVERY:
            modify_input = (
                input("Do you want to add or remove tables? (add/remove/none): ")
                .strip()
                .lower()
            )
            if modify_input == "none":
                break

            if modify_input in ["add", "remove"]:
                table_input = input(
                    f"Enter table number(s) to {modify_input} (comma separated): "
                ).strip()
                tables_to_modify = []
                for table in table_input.split(","):
                    table_name = f"table{table.strip()}"
                    if table_name in WAYPOINTS:
                        tables_to_modify.append(table_name)

                self.lock.acquire()
                if modify_input == "add":
                    for table in tables_to_modify:
                        if table not in self.order_queue:
                            self.order_queue.append(table)
                            rospy.loginfo(f"Added {table} to orders")
                        else:
                            rospy.loginfo(f"{table} is already in the order queue")
                elif modify_input == "remove":
                    for table in tables_to_modify:
                        if table in self.order_queue:
                            self.order_queue.remove(table)
                            rospy.loginfo(f"Removed {table} from orders")
                        else:
                            rospy.loginfo(f"{table} was not found in the order queue")
                self.lock.release()
            else:
                rospy.loginfo("Invalid input, please enter 'add', 'remove', or 'none'")


if __name__ == "__main__":
    try:
        robot_butler = RobotButler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
