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
import rclpy, random, time, os, uuid, json, fcntl, numpy
from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext, GenerationFailedException
from .ConversionTools import ConversionTools as Converter
from .ConversionTools import BadStartingPointError
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
        self._rng_seed = self._initialise_rng()
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
        self.declare_parameter("generator_values_min_straight_variation", 10.0)
        self.declare_parameter("generator_values_max_straight", 80.0)
        self.declare_parameter("generator_values_max_straight_variation", 80.0)
        self.declare_parameter("generator_values_min_constant_turn", 10.0)
        self.declare_parameter("generator_values_min_constant_turn_variation", 10.0)
        self.declare_parameter("generator_values_max_constant_turn", 25.0)
        self.declare_parameter("generator_values_max_constant_turn_variation", 25.0)
        self.declare_parameter("generator_values_min_hairpin", 4.5)
        self.declare_parameter("generator_values_min_hairpin_variation", 4.5)
        self.declare_parameter("generator_values_max_hairpin", 10.0)
        self.declare_parameter("generator_values_max_hairpin_variation", 10.0)
        self.declare_parameter("generator_values_max_hairpin_pairs", 3)
        self.declare_parameter("generator_values_max_hairpin_pairs_variation", 3)
        self.declare_parameter("generator_values_max_length", 1500.0)
        self.declare_parameter("generator_values_max_length_variation", 0.0)
        self.declare_parameter("generator_values_lax_generation", False)
        self.declare_parameter("generator_values_track_width", 3.5)
        self.declare_parameter("generator_values_track_width_variation", 3.5)

        self.declare_parameter("num_tracks_to_generate", 1)
        self.declare_parameter("explore_starting_points", False)
        self.declare_parameter("max_starting_point_error", 0.25)
        self.declare_parameter("write_report", False)
        self.declare_parameter("report_filename", "report.json")
        self.declare_parameter("use_custom_seed", False)
        self.declare_parameter("seed", 0)

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
            "MIN_STRAIGHT_VARIATION": self.get_parameter("generator_values_min_straight_variation").value,
            "MAX_STRAIGHT": self.get_parameter("generator_values_max_straight").value,
            "MAX_STRAIGHT_VARIATION": self.get_parameter("generator_values_max_straight_variation").value,
            "MIN_CONSTANT_TURN": self.get_parameter("generator_values_min_constant_turn").value,
            "MIN_CONSTANT_TURN_VARIATION": self.get_parameter("generator_values_min_constant_turn_variation").value,
            "MAX_CONSTANT_TURN": self.get_parameter("generator_values_max_constant_turn").value,
            "MAX_CONSTANT_TURN_VARIATION": self.get_parameter("generator_values_max_constant_turn_variation").value,
            "MIN_HAIRPIN": self.get_parameter("generator_values_min_hairpin").value,
            "MIN_HAIRPIN_VARIATION": self.get_parameter("generator_values_min_hairpin_variation").value,
            "MAX_HAIRPIN": self.get_parameter("generator_values_max_hairpin").value,
            "MAX_HAIRPIN_VARIATION": self.get_parameter("generator_values_max_hairpin_variation").value,
            "MAX_HAIRPIN_PAIRS": self.get_parameter("generator_values_max_hairpin_pairs").value,
            "MAX_HAIRPIN_PAIRS_VARIATION": self.get_parameter("generator_values_max_hairpin_pairs_variation").value,
            "MAX_LENGTH": self.get_parameter("generator_values_max_length").value,
            "MAX_LENGTH_VARIATION" : self.get_parameter("generator_values_max_length_variation").value,
            "LAX_GENERATION": self.get_parameter("generator_values_lax_generation").value,
            "TRACK_WIDTH": self.get_parameter("generator_values_track_width").value,
            "TRACK_WIDTH_VARIATION": self.get_parameter("generator_values_track_width_variation").value,
            "COMPONENTS": component_data
            }
        other_params = {
            "num_tracks_to_generate" : self.get_parameter("num_tracks_to_generate").value,
            "write_report" : self.get_parameter("write_report").value,
            "report_filename" : self.get_parameter("report_filename").value,
            "use_custom_seed" : self.get_parameter("use_custom_seed").value,
            "seed" : self.get_parameter("seed").value,
            "explore_starting_points" : self.get_parameter("explore_starting_points").value,
            "max_starting_point_error" : self.get_parameter("max_starting_point_error").value,
        }
        return generator_values, other_params

    def _initialise_rng(self):
        """
        Seeds the RNG with system time or a custom seed.
        """
        use_custom = self._node_params["use_custom_seed"]
        custom = self._node_params["seed"]
        seed = custom if use_custom else time.time()
        random.seed(seed)
        numpy.random.seed(int(seed))
        if use_custom: self.get_logger().info(f"Using a custom seed: {custom}")
        return seed
 
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
            fcntl.lockf(f, fcntl.LOCK_EX)
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
            fcntl.lockf(f, fcntl.F_UNLCK)

    """
    Utility function for randomly varying an entire
    parameter set.
    
    :param original_parameter_set: Parameter set.
    :returns: A clone of the original parameter set with the random variations applied.
    """
    def _apply_variations_to_parameter_set(original_parameter_set: dict):
        varied_params = dict(original_parameter_set)
        varied_params["MIN_STRAIGHT"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MIN_STRAIGHT"], varied_params["MIN_STRAIGHT_VARIATION"])
        varied_params["MAX_STRAIGHT"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MAX_STRAIGHT"], varied_params["MAX_STRAIGHT_VARIATION"])
        varied_params["MIN_CONSTANT_TURN"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MIN_CONSTANT_TURN"], varied_params["MIN_CONSTANT_TURN_VARIATION"])
        varied_params["MAX_CONSTANT_TURN"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MAX_CONSTANT_TURN"], varied_params["MAX_CONSTANT_TURN_VARIATION"])
        varied_params["MIN_HAIRPIN"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MIN_HAIRPIN"], varied_params["MIN_HAIRPIN_VARIATION"])
        varied_params["MAX_HAIRPIN"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MAX_HAIRPIN"], varied_params["MAX_HAIRPIN_VARIATION"])
        varied_params["MAX_HAIRPIN_PAIRS"] = int(HeadlessTrackGenerator.randomly_vary_variable(varied_params["MAX_HAIRPIN_PAIRS"], varied_params["MAX_HAIRPIN_PAIRS_VARIATION"]))
        varied_params["MAX_LENGTH"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MAX_LENGTH"], varied_params["MAX_LENGTH_VARIATION"])
        varied_params["MIN_STRAIGHT"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["MIN_STRAIGHT"], varied_params["MIN_STRAIGHT_VARIATION"])
        varied_params["TRACK_WIDTH"] = HeadlessTrackGenerator.randomly_vary_variable(varied_params["TRACK_WIDTH"], varied_params["TRACK_WIDTH_VARIATION"])
        return varied_params

    def _generate_multiple_tracks(self, num_tracks: int, params: dict):
        """
        Generates a specified number of tracks.
        
        :param num_tracks: The number of tracks to generate.
        :param params: The parameters to send to the track generator.
        :returns: A list containing the names of the generated tracks.
        """
        track_names = []
        while len(track_names) < num_tracks:
            track_name = f"{self._pid}_{uuid.uuid4().hex}"
            try:
                # apply variation to parameters
                varied_params = HeadlessTrackGenerator._apply_variations_to_parameter_set(params)
                generated_tracks = HeadlessTrackGenerator.generate_random_track(
                    track_name, varied_params,
                    explore_starting_points=self._node_params["explore_starting_points"],
                    max_starting_point_error=self._node_params["max_starting_point_error"])
                track_names.extend(generated_tracks)
            except GenerationFailedException:
                self.get_logger().error(f"Track generation failed with the following parameters: {varied_params}")
        return track_names

    """
    Utility function to add randomness to a variable.
    
    :param mean: The mean value
    :param range: The size of the range to uniformly sample from. 
    :returns: Result is mean +/- half of range
    """
    def randomly_vary_variable(mean, range):
        return (2.0*random.random() - 1.0)*range + mean

    def convert_generated_track(filename: str, track_data,
                                track_start_component_index: int, max_starting_point_error: float):
        """
        Utility function to attempt to create output files from
        a generated track.
        
        :param filename: The filename, without any extension.
        :param track_data: A dictionary containing the track's path, width and height as consumed
                           by EUFS' conversion tools.
        :raises BadStartingPointException:
        :raises Exception: If the track generator fails for an unecpected reason.
        """
        tracks_folder = get_package_share_directory("eufs_tracks")
        images_folder = os.path.join(
            tracks_folder,
            "image"
        )
        im = Converter.convert(
            "comps",
            "csv",
            filename,
            params=track_data,
            track_start_component_index=track_start_component_index,
            max_starting_point_error=max_starting_point_error,
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

    def generate_random_track(filename: str, generator_values: dict,
                              explore_starting_points=False, max_starting_point_error=0.25):
        """
        This function interfaces with the EUFS random track generator
        to generate a random track and the associated files.
        
        :returns: A list of track names. The list will contain only one element if explore_starting_points is False.
        :param filename: The name of the output files to be generated, minus the extension.
        :param generator_values: A dictionary containing the parameters to use for generation.
        :param explore_starting_points: Whether or not to explore all possible starting points of the random track.
               This includes inverting the path so that the track is driven in the opposite direction.
        :raises RuntimeError: Upon track generation failure.
        """
        track_names = []

        def failure_function():
            raise GenerationFailedException()

        with GeneratorContext(generator_values, failure_function):
            xys, twidth, theight = Generator.generate()
            params = {"track data" : (xys, twidth, theight)}
            if explore_starting_points:
                for i in range(len(xys)):
                    track_name = f"{filename}_{i}"
                    try:
                        HeadlessTrackGenerator.convert_generated_track(
                            track_name,
                            params,
                            i,
                            max_starting_point_error
                        ) 
                        track_names.append(track_name)
                    except BadStartingPointError:
                        print(f"{track_name} failed.")
                    except Exception as e:
                        print(f"{track_name} failed for an unexpected reason. Exception: {e}")
            else:
                try:
                    HeadlessTrackGenerator.convert_generated_track(
                        filename,
                        params,
                        0,
                        max_starting_point_error
                    )
                    track_names.append(filename)
                except BadStartingPointError:
                    print(f"{filename} failed.")
                except Exception as e:
                    print(f"{filename} failed for an unexpected reason. Exception: {e}")

        return track_names

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessTrackGenerator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
