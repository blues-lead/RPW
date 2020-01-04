# Code for Robotics Project Work
The task is to get a car autonomously driving in the Unreal Engine "AirSim" environment

* testAPI.py - code for testing AirSim APIs
* custom_env.py - custom environment. Extends OpenAI's env class in order to meet OpenAI. This is  needed for using PPO and TRPO algorithms in the stable_baselines library

Teh custom_env is done using virtual environment, because there some package conflicts:
* version of the tensorflow must be between 1.8.0 and 1.14.0
* Version of msgpack-python must be 0.5.6
* Version of msgpack-rpc-python msut be 0.4.1

The environment used for testing is Blocks/AirSim.
