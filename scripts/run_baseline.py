import argparse
import datetime
import os
import random
import subprocess


SIMULATOR_JAR = os.getenv("SIMULATOR_JAR")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp-name", type=str, required=True,
                        help="name of the experiment")
    parser.add_argument("--scripted-spawn", type=str, default="",
                        help="path to CSV file containing custom aircraft spawn instructions")
    args = parser.parse_args()

    # Start trainer process first, pin to first core if possible
    env_id = f"0_{random.randbytes(3).hex()}"

    # Start simulator process
    sim_args = ["java", "-jar", SIMULATOR_JAR]
    taskset_args = []
    output_path = f"runs/{args.exp_name}_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    all_args = (
            taskset_args + sim_args +
            [
                env_id, "0", "4", "8", "1", "1", "0", args.scripted_spawn, output_path
            ]
    )
    print("Launching", " ".join(all_args))
    process = subprocess.Popen(all_args)
    process.wait()
