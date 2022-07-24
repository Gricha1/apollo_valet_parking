def update_car_config():
    car_config = \
    {
        "length": 4.2,
        "width": 1.8,
        "wheel_base": 2.8448,
        "safe_eps": 0.001,
        "max_steer": 28,
        "max_vel": 2,
        "min_vel": -2,
        "max_acc": 1.5,
        "max_ang_vel": 1,
        "max_ang_acc": 1.5,
        "delta_t": 0.1,
        "use_clip": 1,
        "jerk": 0.5
    }

    return car_config

def update_our_env_config():
    our_env_config = \
    {
        "empty": 1,
        "obstacles": 1,
        "dyn_obstacles": 0,
        "alpha": 45,
        "discrete_alpha": 11,
        "max_steer": 10,
        "max_dist": 30,
        "min_dist": 15,
        "min_vel": 0,
        "max_vel": 10,
        "min_obs_v": 0,
        "max_obs_v": 0,
        "UPDATE_SPARSE": 8,
        "HARD_EPS": 0.8,
        "MEDIUM_EPS": 0.8,
        "SOFT_EPS": 1.5,
        "ANGLE_EPS": 15,
        "SPEED_EPS": 1,
        "STEERING_EPS": 20,
        "view_angle": 180,
        "MAX_DIST_LIDAR": 20,
        "bias_beam": 0,
        "n_beams": 39,
        "frame_stack": 4,
        "hard_constraints": 1,
        "medium_constraints": 0,
        "soft_constraints": 0,
        "affine_transform": 0,
        "reward_with_potential": 1,
        "use_acceleration_penalties" : 1,
        "use_velocity_goal_penalty" : 0,
        "union": 1,
        "dynamic": 1,
        "static": 1,
        "easy_map_constraints": 0,
        "medium_map_constraints": 0,
        "hard_map_constraints": 1,
        "validate_custom_case" : 0,
        "easy_task_difficulty": 0,
        "medium_task_difficulty": 0,
        "hard_task_difficulty": 1
    }
    return our_env_config


def update_train_config():
    train_config = \
    {
        "framework": "torch",
        "wandb" : 1,
        "train_batch_size": 8000,
        "lambda": 0.96,
        "use_critic": 1,
        "use_gae": 1,
        "horizon": 300,
        "rollout_fragment_length": 1600,
        "num_gpus": 1,
        "num_workers": 5,
        "lr": 0.0001,
        "sgd_minibatch_size": 512,
        "num_sgd_iter": 10,
        "clip_param": 0.2,
        "optimizer": "adam",
        "entropy_coeff": 0.01,
        "vf_clip_param": 100,
        "normalize_actions": 0,
        "batch_mode": "complete_episodes",
        "max_episodes": 100000,
        "val_time_steps": 250,
        "acceptable_success": 80,
        "name_save": "rllib_ppoNew3",
        "curriculum_name": "rllib_ppoWithOrientation",
        "project_name": "POLAMP_ppo",
        "curriculum": 0,
        "steps_curriculum": 1,
        "_fake_gpus": 1,
        "vf_loss_coeff": 0.1,
        "use_lstm": 0,
        "lstm_use_prev_action": 0,
        "free_log_std": 1,
        "run_num": 60,
        "ex_num": 2,
        "conf_num": 1,
        "loaded_run_num": 20,
        "loaded_ex_num": 2,
        "loaded_conf_num": 1,
        "loaded_check_num": 4910,
        "no_initial_weights": 0
    }

    return train_config


def update_reward_config():
    reward_config = \
    {
        "collision": 20,
        "goal": 200,
        "timeStep": 1,
        "distance": 5,
        "overSpeeding": 5,
        "overSteering": 5,
        "Eps_penalty": 0.5,
        "a_penalty": 0.5,
        "v_goal_penalty": 30
    }   
    return reward_config