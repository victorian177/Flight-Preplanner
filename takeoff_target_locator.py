import json
import logging
import os


class TakeoffTargetLocator:
    FOLDER = "takeoff_target_locations"

    def __init__(self) -> None:
        pass

    def list_missions(self):
        missions = os.listdir(self.FOLDER)
        logging.info("Saved Missions:")
        logging.info(missions)
        print(missions)
        logging.info("")

    def add_mission(self, mssn=None):
        mission = {}

        if mssn is not None:
            mission = mssn
        else:
            mission["takeoff_lat"] = float(input("Enter takeoff latitude: "))
            mission["takeoff_lon"] = float(input("Enter takeoff longitude: "))
            mission["target_lat"] = float(input("Enter target latitude: "))
            mission["target_lon"] = float(input("Enter target longitude: "))

        if (
            (-90.0 < mission["takeoff_lat"] < 90.0)
            and (-90.0 < mission["target_lat"] < 90.0)
            and (-180.0 < mission["takeoff_lon"] < 180.0)
            and (-180.0 < mission["target_lon"] < 180.0)
        ):
            self.list_missions()
            mission_name = input("Enter mission name: ")
            filepath = f"{self.FOLDER}/{mission_name}.json"

            with open(filepath, "w") as mission_file:
                json.dump(mission, mission_file)

            logging.info(f"{mission_name} saved!")
            logging.info("")

        else:
            logging.info(
                """
            At least one of the values entered is not valid.
            Latitude values should be between -90.0 degrees and 90.0 degrees.
            Longitude values should be between -180.0 degrees and 180.0 degrees.
            """
            )

    def remove_mission(self):
        self.list_missions()
        mission_name = input("Enter mission name: ")
        filepath = f"{self.FOLDER}/{mission_name}.json"

        if os.path.isfile(filepath):
            os.remove(filepath)
            logging.info(f"{filepath} has been removed!")
        else:
            logging.info(f"{filepath} does not exist!")

    def load_mission(self):
        self.list_missions()
        mission_name = input("Enter mission name: ")
        filepath = f"{self.FOLDER}/{mission_name}.json"

        file = None

        if os.path.isfile(filepath):
            with open(filepath, "r") as mission_file:
                file = json.load(mission_file)
        else:
            logging.info(f"{filepath} does not exist!")

        return file

    def change_mission(self):
        mission = self.load_mission()

        print("Previous mission values:")
        print(mission)

        mission["takeoff_lat"] = float(input("Enter takeoff latitude: "))
        mission["takeoff_lon"] = float(input("Enter takeoff longitude: "))
        mission["target_lat"] = float(input("Enter target latitude: "))
        mission["target_lon"] = float(input("Enter target longitude: "))

        self.add_mission(mission)


if __name__ == "__main__":
    test = TakeoffTargetLocator()
    test.list_missions()

    test.add_mission()
