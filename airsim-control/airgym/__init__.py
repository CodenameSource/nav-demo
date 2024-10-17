from gymnasium.envs.registration import register

register(
    id='airsim-drone-sample-v0',
    entry_point='airgym.env:AirsimDroneEnv',
)