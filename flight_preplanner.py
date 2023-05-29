import time

from pymavlink import mavutil

from takeoff_target_locator import TakeoffTargetLocator


class FlightPreplanner:
    def __init__(self) -> None:
        self.mission_coordinates = self.load_mission_coordinates()
        self.vehicle = None
        self.setup()

    def load_mission_coordinates(self):
        ttl = TakeoffTargetLocator()
        mission_coordinates = ttl.load_mission()

        return mission_coordinates

    def setup(self):
        self.vehicle = mavutil.mavlink_connection("udp:127.0.0.1:14550")

        print("Established connection.")

        while True:
            msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)
            if msg.get_type() == "HEARTBEAT":
                print("Gotten heartbeat.")
                break

        self.vehicle.set_mode_guided()

        while not self.vehicle.location.global_relative_frame.lat:
            time.sleep(0.5)

        print("Done")

    def upload_mission(self):
        self.vehicle.mav.mission_clear_all_send(
            self.vehicle.target_system, self.vehicle.target_component
        )
        self.vehicle.mav.mission_count_send(
            self.vehicle.target_system, self.vehicle.target_component, 2
        )

        self.vehicle.mav.mission_item_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            0,  # sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # autocontinue
            0,  # param1 (min pitch)
            0,  # param2 (empty)
            0,  # param3 (empty)
            0,  # param4 (yaw angle)
            self.mission_coordinates["takeoff_lat"],  # x (latitude)
            self.mission_coordinates["takeoff_lon"],  # y (longitude)
            10,  # z (takeoff altitude in meters)
        )

        self.vehicle.mav.mission_item_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            1,  # sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # autocontinue
            0,  # param1 (hold time)
            0,  # param2 (empty)
            0,  # param3 (empty)
            0,  # param4 (yaw angle)
            self.mission_coordinates["target_lat"],  # x (latitude)
            self.mission_coordinates["target_lon"],  # y,  # z (altitude)
        )

        self.vehicle.mav.mission_write_partial_list_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            0,  # start index
            1,  # end index
        )

    def arm(self):
        self.vehicle.set_mode_auto()
        self.vehicle.arducopter_arm()

    def takeoff(self):
        # Wait for the drone to reach the takeoff location
        while (
            abs(self.vehicle.location.lat - self.mission_coordinates["takeoff_lat"])
            > 0.00005
            or abs(self.vehicle.location.lon - self.mission_coordinates["takeoff_lon"])
            > 0.00005
        ):
            time.sleep(0.5)

    def target(self):
        while (
            abs(self.vehicle.location.lat - self.mission_coordinates["target_lat"])
            > 0.00005
            or abs(self.vehicle.location.lon - self.mission_coordinates["target_lon"])
            > 0.00005
        ):
            time.sleep(0.5)

    def disarm(self):
        self.vehicle.set_mode_loiter()
        self.vehicle.arducopter_disarm()


if __name__ == "__main__":
    fpp = FlightPreplanner()
    fpp.setup()