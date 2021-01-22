use crate::{Error, RobotClient};
use arci::{JointTrajectoryClient, MoveBase, Navigation, Speaker};
#[cfg(feature = "ros")]
use arci_ros::{
    RosCmdVelMoveBase, RosCmdVelMoveBaseConfig, RosControlClientConfig, RosEspeakClient,
    RosEspeakClientConfig, RosNavClient, RosNavClientConfig,
};
use arci_urdf_viz::{create_joint_trajectory_clients, UrdfVizWebClient, UrdfVizWebClientConfig};

use openrr_client::{CollisionCheckClientConfig, IkClientConfig, PrintSpeaker};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::PathBuf, sync::Arc};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RobotConfig {
    #[cfg(feature = "ros")]
    pub ros_clients_configs: Vec<RosControlClientConfig>,
    pub urdf_viz_clients_configs: Vec<UrdfVizWebClientConfig>,

    pub urdf_viz_clients_total_complete_allowable_error: f64,
    pub urdf_viz_clients_complete_timeout_sec: f64,

    #[cfg(feature = "ros")]
    pub ros_espeak_client_config: Option<RosEspeakClientConfig>,

    #[cfg(feature = "ros")]
    pub ros_cmd_vel_move_base_client_config: Option<RosCmdVelMoveBaseConfig>,
    pub use_move_base_urdf_viz_web_client: bool,

    #[cfg(feature = "ros")]
    pub ros_navigation_client_config: Option<RosNavClientConfig>,
    pub use_navigation_urdf_viz_web_client: bool,

    pub urdf_path: String,
    urdf_full_path: Option<PathBuf>,
    pub self_collision_check_pairs: Vec<String>,

    pub collision_check_clients_configs: Vec<CollisionCheckClientConfig>,
    pub ik_clients_configs: Vec<IkClientConfig>,

    pub joints_poses: Vec<JointsPose>,
}

impl RobotConfig {
    pub fn create_robot_client<S, M, N>(&self) -> Result<RobotClient<S, M, N>, Error>
    where
        S: Speaker + From<Box<dyn Speaker>>,
        M: MoveBase + From<Box<dyn MoveBase>>,
        N: Navigation + From<Box<dyn Navigation>>,
    {
        RobotClient::try_new(
            self.clone(),
            self.create_raw_joint_trajectory_clients()?,
            self.create_speaker().into(),
            self.create_move_base()?.map(|m| m.into()),
            self.create_navigation()?.map(|n| n.into()),
        )
    }
    fn create_navigation_without_ros(&self) -> Result<Option<Box<dyn Navigation>>, Error> {
        Ok(if self.use_navigation_urdf_viz_web_client {
            let urdf_viz_client = Box::new(UrdfVizWebClient::default());
            urdf_viz_client.run_thread();
            Some(urdf_viz_client as Box<dyn Navigation>)
        } else {
            None
        })
    }
    #[cfg(feature = "ros")]
    fn create_navigation_with_ros(&self) -> Result<Option<Box<dyn Navigation>>, Error> {
        if let Some(ros_navigation_client_config) = &self.ros_navigation_client_config {
            Ok(Some(Box::new(RosNavClient::new_from_config(
                ros_navigation_client_config.clone(),
            )) as Box<dyn Navigation>))
        } else {
            self.create_navigation_without_ros()
        }
    }
    fn create_navigation(&self) -> Result<Option<Box<dyn Navigation>>, Error> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_navigation_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_navigation_with_ros()
        }
    }
    fn create_move_base_without_ros(&self) -> Result<Option<Box<dyn MoveBase>>, Error> {
        Ok(if self.use_move_base_urdf_viz_web_client {
            let urdf_viz_client = Box::new(UrdfVizWebClient::default());
            urdf_viz_client.run_thread();
            Some(urdf_viz_client as Box<dyn MoveBase>)
        } else {
            None
        })
    }
    #[cfg(feature = "ros")]
    fn create_move_base_with_ros(&self) -> Result<Option<Box<dyn MoveBase>>, Error> {
        if let Some(ros_cmd_vel_move_base_client_config) = &self.ros_cmd_vel_move_base_client_config
        {
            Ok(Some(Box::new(RosCmdVelMoveBase::new(
                &ros_cmd_vel_move_base_client_config.topic,
            )) as Box<dyn MoveBase>))
        } else {
            self.create_move_base_without_ros()
        }
    }
    fn create_move_base(&self) -> Result<Option<Box<dyn MoveBase>>, Error> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_move_base_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_move_base_with_ros()
        }
    }
    fn create_speaker_without_ros(&self) -> Box<dyn Speaker> {
        Box::new(PrintSpeaker::new())
    }
    #[cfg(feature = "ros")]
    fn create_speaker_with_ros(&self) -> Box<dyn Speaker> {
        if let Some(c) = &self.ros_espeak_client_config {
            Box::new(RosEspeakClient::new(&c.topic))
        } else {
            self.create_speaker_without_ros()
        }
    }
    fn create_speaker(&self) -> Box<dyn Speaker> {
        #[cfg(not(feature = "ros"))]
        {
            self.create_speaker_without_ros()
        }
        #[cfg(feature = "ros")]
        {
            self.create_speaker_with_ros()
        }
    }
    fn create_raw_joint_trajectory_clients(
        &self,
    ) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, Error> {
        #[cfg(not(feature = "ros"))]
        let raw_joint_trajectory_clients = if self.urdf_viz_clients_configs.is_empty() {
            return Err(Error::NoClientsConfigs("urdf_viz_clients".to_owned()));
        } else {
            create_joint_trajectory_clients(
                self.urdf_viz_clients_configs.clone(),
                self.urdf_viz_clients_total_complete_allowable_error,
                self.urdf_viz_clients_complete_timeout_sec,
            )
        };
        #[cfg(feature = "ros")]
        let raw_joint_trajectory_clients = {
            let mut clients = if self.urdf_viz_clients_configs.is_empty() {
                HashMap::new()
            } else {
                create_joint_trajectory_clients(
                    self.urdf_viz_clients_configs.clone(),
                    self.urdf_viz_clients_total_complete_allowable_error,
                    self.urdf_viz_clients_complete_timeout_sec,
                )
            };
            clients.extend(
                arci_ros::create_joint_trajectory_clients(self.ros_clients_configs.clone())
                    .into_iter(),
            );
            if clients.is_empty() {
                return Err(Error::NoClientsConfigs(
                    "urdf_viz_clients_configs / ros_clients_configs".to_owned(),
                ));
            }
            clients
        };
        Ok(raw_joint_trajectory_clients)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct JointsPose {
    pub pose_name: String,
    pub client_name: String,
    pub positions: Vec<f64>,
}

impl RobotConfig {
    pub fn try_new<P: AsRef<std::path::Path>>(path: P) -> Result<Self, Error> {
        let mut config: RobotConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        config.urdf_full_path = Some(
            path.as_ref()
                .parent()
                .ok_or_else(|| Error::NoParentDirectory(path.as_ref().to_owned()))?
                .join(&config.urdf_path),
        );
        Ok(config)
    }
    pub fn urdf_full_path(&self) -> &Option<PathBuf> {
        &self.urdf_full_path
    }
}