use openrr_apps::RobotConfig;

#[test]
fn verify_sample_configs() {
    let files = vec![
        "config/pr2_robot_client_config_for_ros.toml",
        "config/pr2_robot_client_config_for_urdf_viz.toml",
        "config/sample_robot_client_config_for_urdf_viz.toml",
        "config/ur10_robot_client_config_for_ros.toml",
        "config/ur10_robot_client_config_for_urdf_viz.toml",
    ];
    for f in files {
        let result = RobotConfig::try_new(f);
        assert!(result.is_ok(), "{:?} {:?}", f, result);
    }
}