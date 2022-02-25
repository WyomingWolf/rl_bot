# *******************************************************************************
#  test_env.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 08/21/2021
# *******************************************************************************


from bots import ant
from stable_baselines3.common.env_checker import check_env

env = ant()
# It will check your custom environment and output additional warnings if needed
check_env(env)
print("Enviornment check complete")
env.stop()

