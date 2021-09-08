// This file is @generated by openrr-internal-codegen.
// It is not intended for manual editing.

#![allow(unused_variables)]
#![allow(clippy::useless_conversion, clippy::unit_arg)]

use arci::{BaseVelocity, Error, Isometry2, Isometry3, WaitFuture};

use super::*;
#[derive(Debug, Clone)]
pub struct RemoteGamepadSender {
    pub(crate) client: pb::gamepad_client::GamepadClient<tonic::transport::Channel>,
}
impl RemoteGamepadSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::gamepad_client::GamepadClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::gamepad_client::GamepadClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteGamepadReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteGamepadReceiver<T>
where
    T: arci::Gamepad + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::gamepad_server::GamepadServer<Self> {
        pb::gamepad_server::GamepadServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
#[derive(Debug, Clone)]
pub struct RemoteJointTrajectoryClientSender {
    pub(crate) client:
        pb::joint_trajectory_client_client::JointTrajectoryClientClient<tonic::transport::Channel>,
}
impl RemoteJointTrajectoryClientSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::joint_trajectory_client_client::JointTrajectoryClientClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::joint_trajectory_client_client::JointTrajectoryClientClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteJointTrajectoryClientReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteJointTrajectoryClientReceiver<T>
where
    T: arci::JointTrajectoryClient + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(
        self,
    ) -> pb::joint_trajectory_client_server::JointTrajectoryClientServer<Self> {
        pb::joint_trajectory_client_server::JointTrajectoryClientServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
#[derive(Debug, Clone)]
pub struct RemoteLocalizationSender {
    pub(crate) client: pb::localization_client::LocalizationClient<tonic::transport::Channel>,
}
impl RemoteLocalizationSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::localization_client::LocalizationClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::localization_client::LocalizationClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteLocalizationReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteLocalizationReceiver<T>
where
    T: arci::Localization + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::localization_server::LocalizationServer<Self> {
        pb::localization_server::LocalizationServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
impl arci::Localization for RemoteLocalizationSender {
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new(frame_id.into());
        Ok(block_in_place(client.current_pose(args))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .into())
    }
}
#[tonic::async_trait]
impl<T> pb::localization_server::Localization for RemoteLocalizationReceiver<T>
where
    T: arci::Localization + 'static,
{
    async fn current_pose(
        &self,
        request: tonic::Request<::prost::alloc::string::String>,
    ) -> Result<tonic::Response<pb::Isometry2>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::Localization::current_pose(&self.inner, &request)
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .into();
        Ok(tonic::Response::new(res))
    }
}
#[derive(Debug, Clone)]
pub struct RemoteMoveBaseSender {
    pub(crate) client: pb::move_base_client::MoveBaseClient<tonic::transport::Channel>,
}
impl RemoteMoveBaseSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::move_base_client::MoveBaseClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::move_base_client::MoveBaseClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteMoveBaseReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteMoveBaseReceiver<T>
where
    T: arci::MoveBase + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::move_base_server::MoveBaseServer<Self> {
        pb::move_base_server::MoveBaseServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
impl arci::MoveBase for RemoteMoveBaseSender {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new((*velocity).into());
        Ok(block_in_place(client.send_velocity(args))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .into())
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new(());
        Ok(block_in_place(client.current_velocity(args))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .into())
    }
}
#[tonic::async_trait]
impl<T> pb::move_base_server::MoveBase for RemoteMoveBaseReceiver<T>
where
    T: arci::MoveBase + 'static,
{
    async fn send_velocity(
        &self,
        request: tonic::Request<pb::BaseVelocity>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::MoveBase::send_velocity(&self.inner, &request.into())
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .into();
        Ok(tonic::Response::new(res))
    }

    async fn current_velocity(
        &self,
        request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::BaseVelocity>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::MoveBase::current_velocity(&self.inner)
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .into();
        Ok(tonic::Response::new(res))
    }
}
#[derive(Debug, Clone)]
pub struct RemoteNavigationSender {
    pub(crate) client: pb::navigation_client::NavigationClient<tonic::transport::Channel>,
}
impl RemoteNavigationSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::navigation_client::NavigationClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::navigation_client::NavigationClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteNavigationReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteNavigationReceiver<T>
where
    T: arci::Navigation + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::navigation_server::NavigationServer<Self> {
        pb::navigation_server::NavigationServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
impl arci::Navigation for RemoteNavigationSender {
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new((goal, frame_id, timeout).into());
        Ok(wait_from_handle(tokio::spawn(async move {
            client.send_goal_pose(args).await
        })))
    }

    fn cancel(&self) -> Result<(), Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new(());
        Ok(block_in_place(client.cancel(args))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .into())
    }
}
#[tonic::async_trait]
impl<T> pb::navigation_server::Navigation for RemoteNavigationReceiver<T>
where
    T: arci::Navigation + 'static,
{
    async fn send_goal_pose(
        &self,
        request: tonic::Request<pb::GoalPoseRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::Navigation::send_goal_pose(
            &self.inner,
            request.goal.unwrap().into(),
            &request.frame_id,
            request.timeout.unwrap().try_into().unwrap(),
        )
        .map_err(|e| tonic::Status::unknown(e.to_string()))?
        .await
        .map_err(|e| tonic::Status::unknown(e.to_string()))?
        .into();
        Ok(tonic::Response::new(res))
    }

    async fn cancel(
        &self,
        request: tonic::Request<()>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::Navigation::cancel(&self.inner)
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .into();
        Ok(tonic::Response::new(res))
    }
}
#[derive(Debug, Clone)]
pub struct RemoteSpeakerSender {
    pub(crate) client: pb::speaker_client::SpeakerClient<tonic::transport::Channel>,
}
impl RemoteSpeakerSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::speaker_client::SpeakerClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::speaker_client::SpeakerClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteSpeakerReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteSpeakerReceiver<T>
where
    T: arci::Speaker + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::speaker_server::SpeakerServer<Self> {
        pb::speaker_server::SpeakerServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
impl arci::Speaker for RemoteSpeakerSender {
    fn speak(&self, message: &str) -> Result<WaitFuture, Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new(message.into());
        Ok(wait_from_handle(tokio::spawn(async move {
            client.speak(args).await
        })))
    }
}
#[tonic::async_trait]
impl<T> pb::speaker_server::Speaker for RemoteSpeakerReceiver<T>
where
    T: arci::Speaker + 'static,
{
    async fn speak(
        &self,
        request: tonic::Request<::prost::alloc::string::String>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::Speaker::speak(&self.inner, &request)
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .await
            .map_err(|e| tonic::Status::unknown(e.to_string()))?
            .into();
        Ok(tonic::Response::new(res))
    }
}
#[derive(Debug, Clone)]
pub struct RemoteTransformResolverSender {
    pub(crate) client:
        pb::transform_resolver_client::TransformResolverClient<tonic::transport::Channel>,
}
impl RemoteTransformResolverSender {
    /// Attempt to create a new sender by connecting to a given endpoint.
    pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
    {
        let client = pb::transform_resolver_client::TransformResolverClient::connect(dst)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(Self { client })
    }

    /// Create a new sender.
    pub fn new(channel: tonic::transport::Channel) -> Self {
        Self {
            client: pb::transform_resolver_client::TransformResolverClient::new(channel),
        }
    }
}
#[derive(Debug)]
pub struct RemoteTransformResolverReceiver<T> {
    pub(crate) inner: T,
}
impl<T> RemoteTransformResolverReceiver<T>
where
    T: arci::TransformResolver + 'static,
{
    /// Create a new receiver.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Convert this receiver into a tower service.
    pub fn into_service(self) -> pb::transform_resolver_server::TransformResolverServer<Self> {
        pb::transform_resolver_server::TransformResolverServer::new(self)
    }

    pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
        tonic::transport::Server::builder()
            .add_service(self.into_service())
            .serve(addr)
            .await
            .map_err(|e| arci::Error::Connection {
                message: e.to_string(),
            })?;
        Ok(())
    }
}
impl arci::TransformResolver for RemoteTransformResolverSender {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: std::time::SystemTime,
    ) -> Result<Isometry3<f64>, Error> {
        let mut client = self.client.clone();
        let args = tonic::Request::new((from, to, time).into());
        Ok(block_in_place(client.resolve_transformation(args))
            .map_err(|e| arci::Error::Other(e.into()))?
            .into_inner()
            .into())
    }
}
#[tonic::async_trait]
impl<T> pb::transform_resolver_server::TransformResolver for RemoteTransformResolverReceiver<T>
where
    T: arci::TransformResolver + 'static,
{
    async fn resolve_transformation(
        &self,
        request: tonic::Request<pb::ResolveTransformationRequest>,
    ) -> Result<tonic::Response<pb::Isometry3>, tonic::Status> {
        let request = request.into_inner();
        let res = arci::TransformResolver::resolve_transformation(
            &self.inner,
            &request.from,
            &request.to,
            request.time.unwrap().try_into().unwrap(),
        )
        .map_err(|e| tonic::Status::unknown(e.to_string()))?
        .into();
        Ok(tonic::Response::new(res))
    }
}