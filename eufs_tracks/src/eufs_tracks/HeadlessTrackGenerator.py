##===-- HeadlessTrackGenerator.py - HeadlessTrackGenerator -*- py -*-===##
##
## Part of the UGRDV Project, under the UGRACING DRIVERLESS PRIVATE LICENSE.
## See the attached LICENSE.txt file in this projects upper most directory for 
## license information.
##
## Copyright Joseph Agrane © 2023
##
##===----------------------------------------------------------------------===##
###
### \file
### A headless interface to the EUFS Sim random track generator
###
##===----------------------------------------------------------------------===##
from rclpy.node import Node
import rclpy, random, time, os, uuid, json, numpy
from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext
from .ConversionTools import ConversionTools as Converter
from ament_index_python import get_package_share_directory

class HeadlessTrackGenerator(Node):
    def __init__(self):
        """
        This node is responsible for interfacing with the
        EUFS track generator and generating random tracks.
        """
        super().__init__("headless_track_generator")
        self._pid = os.getpid()
        self._declare_ros_parameters()
        self._generator_values, self._node_params = self._read_ros_parameters()
        self._initialise_rng(self._pid)
        self._run()

    def _run(self):
        self._log_status()
        tracks = self._generate_multiple_tracks(self._node_params["num_tracks_to_generate"], self._generator_values)
        if self._node_params["write_report"] == True:
            self._write_report(self._node_params["report_filename"], tracks, self._rng_seed, self._pid, self._generator_values)

    def _declare_ros_parameters(self):
        """
        This function declares all of the parameters that it expects
        to receive from ROS.
        """
        self.declare_parameter("component_data_straight", 1.0)
        self.declare_parameter("component_data_constant_turn", 0.7)
        self.declare_parameter("component_data_hairpin_turn", 0.3)

        self.declare_parameter("generator_values_min_straight", 10.0)
        self.declare_parameter("generator_values_max_straight", 80.0)
        self.declare_parameter("generator_values_min_constant_turn", 10.0)
        self.declare_parameter("generator_values_max_constant_turn", 25.0)
        self.declare_parameter("generator_values_min_hairpin", 4.5)
        self.declare_parameter("generator_values_max_hairpin", 10.0)
        self.declare_parameter("generator_values_max_hairpin_pairs", 3)
        self.declare_parameter("generator_values_max_length", 1500.0)
        self.declare_parameter("generator_values_max_length_variation", 0.0)
        self.declare_parameter("generator_values_lax_generation", False)
        self.declare_parameter("generator_values_track_width", 3.5)

        self.declare_parameter("num_tracks_to_generate", 1)
        self.declare_parameter("write_report", False)
        self.declare_parameter("report_filename", "report.json")

    def _read_ros_parameters(self):
        """
        Reads expected ROS parameters.

        :returns: Tuple (2,) in which the first element contains all parameters to be consumed
        by the track generator and the second element contains any other parameters.
        """
        component_data = {
            "STRAIGHT" : self.get_parameter("component_data_straight").value,
            "CONSTANT_TURN": self.get_parameter("component_data_constant_turn").value,
            "HAIRPIN_TURN" : self.get_parameter("component_data_hairpin_turn").value,
        }
        generator_values = {
            "MIN_STRAIGHT": self.get_parameter("generator_values_min_straight").value,
            "MAX_STRAIGHT": self.get_parameter("generator_values_max_straight").value,
            "MIN_CONSTANT_TURN": self.get_parameter("generator_values_min_constant_turn").value,
            "MAX_CONSTANT_TURN": self.get_parameter("generator_values_max_constant_turn").value,
            "MIN_HAIRPIN": self.get_parameter("generator_values_min_hairpin").value,
            "MAX_HAIRPIN": self.get_parameter("generator_values_max_hairpin").value,
            "MAX_HAIRPIN_PAIRS": self.get_parameter("generator_values_max_hairpin_pairs").value,
            "MAX_LENGTH": self.get_parameter("generator_values_max_length").value,
            "MAX_LENGTH_VARIATION" : self.get_parameter("generator_values_max_length_variation").value,
            "LAX_GENERATION": self.get_parameter("generator_values_lax_generation").value,
            "TRACK_WIDTH": self.get_parameter("generator_values_track_width").value,
            "COMPONENTS": component_data
            }
        other_params = {
            "num_tracks_to_generate" : self.get_parameter("num_tracks_to_generate").value,
            "write_report" : self.get_parameter("write_report").value,
            "report_filename" : self.get_parameter("report_filename").value,
        }
        return generator_values, other_params

    def _initialise_rng(self, pid: int):
        """
        Seeds the RNG with {pid}_{system_time} (hashed)

        :param pid: The pid.
        """
        numpy_max_seed = 2**32 - 1
        seed = abs(hash(f"{pid}_{int(time.time())}")) % (numpy_max_seed + 1)
        self._rng_seed = seed
        random.seed(seed)
        numpy.random.seed(seed)
 
    def _log_status(self):
        l = self.get_logger()
        l.info(f"Running with pid: {self._pid}, seed; {self._rng_seed}")
        l.info(f"Set to generate {self._node_params['num_tracks_to_generate']} tracks.")

    def _write_report(self, filename, tracks_generated: list, seed_used: int, pid: int, params: dict):
        """
        Appends to a json report information about the tracks which were generated.
        
        :param filename: The output filename for the report json file.
        :param tracks_generated: A list contianing the names of the tracks which were generated.
        :param seed_used: The seed which was used to generate the tracks.
        :param pid: The PID.
        :param params: The parameters used to generate the tracks.
        """
        path = os.path.join(
            get_package_share_directory("eufs_tracks"),
            filename
        )
        new_report = not os.path.exists(path)
        with open(path, "r+") if not new_report else open(path, "w") as f:
            report = []
            if not new_report:
                report = json.load(f)
                f.truncate(0)
                f.seek(0)
            report.append({
                "pid" : pid,
                "seed" : seed_used,
                "tracks" : tracks_generated,
                "parameters" : params
            })
            json.dump(report, f)

    def _generate_multiple_tracks(self, num_tracks: int, params: dict):
        """
        Generates a specified number of tracks.
        
        :param num_tracks: The number of tracks to generate.
        :param params: The parameters to send to the track generator.
        :returns: A list containing the names of the generated tracks.
        """
        track_names = []
        original_track_length = params["MAX_LENGTH"]
        vary_track_length = params["MAX_LENGTH_VARIATION"] >= 1.0
        for i in range(num_tracks):
            track_name = f"{self._pid}_{uuid.uuid4().hex}"
            try:
                # optionally apply some variation to maximum track length
                if vary_track_length:
                    params["MAX_LENGTH"] = HeadlessTrackGenerator.randomly_vary_variable(
                        original_track_length,
                        params["MAX_LENGTH_VARIATION"])
                else:
                    params["MAX_LENGTH"] = original_track_length
                print(f"max_length {params['MAX_LENGTH']}")
                HeadlessTrackGenerator.generate_random_track(track_name, params)
                track_names.append(track_name)
            except RuntimeError:
                self.get_logger().error(f"Track generation failed with the following parameters: {params}")
        return track_names

    """
    Utility function to add randomness to a variable.
    
    :param mean: The mean value
    :param range: The size of the range to uniformly sample from. 
    :returns: Result is mean +/- half of range
    """
    def randomly_vary_variable(mean, range):
        return (2.0*random.random() - 1.0)*range + mean

    def generate_random_track(filename: str, generator_values: dict):
        """
        This function interfaces with the EUFS random track generator
        to generate a random track and the associated files.
        
        :param filename: The name of the output files to be generated, minus the extension.
        :param generator_values: A dictionary containing the parameters to use for generation.
        :raises RuntimeError: Upon track generation failure.
        """
        tracks_folder = get_package_share_directory("eufs_tracks")
        images_folder = os.path.join(tracks_folder, "image")

        def failure_function():
            raise RuntimeError()

        with GeneratorContext(generator_values, failure_function):
            xys, twidth, theight = Generator.generate()
            im = Converter.convert(
                "comps",
                "csv",
                filename,
                params={
                    "track data": (xys, twidth, theight)
                }
            )
            csv_path = os.path.join(
                tracks_folder,
                f"csv/{filename}.csv"
            )
            Converter.convert(
                "csv",
                "ALL",
                csv_path,
                params={"noise" : 0.001}
            )
            im.save(os.path.join(images_folder, f"{filename}.png"))

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessTrackGenerator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
