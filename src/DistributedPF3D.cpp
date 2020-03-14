//
// Created by glawless on 23.05.17.
//

#include <target_tracker_distributed_pf/DistributedPF3D.h>
#include <ros/callback_queue.h>
#include <random>
#include <eigen3/Eigen/Cholesky>

namespace target_tracker_distributed_pf {

static const std::string world_frame{"world"};

DistributedPF3D::DistributedPF3D() : pnh_("~") {
    // Advertise dynamic reconfigure server
    dynamic_reconfigure::Server<ParticleFilterParamsConfig>::CallbackType cb(
        boost::bind(&DistributedPF3D::dynamicReconfigureCallback, this, _1, _2));
    dyn_rec_server_.setCallback(cb);

    // Some parameters related to initial uncertainty
    pnh_.getParam("initialUncertaintyPosXY", initialUncertaintyPosXY);
    pnh_.getParam("initialUncertaintyPosZ", initialUncertaintyPosZ);
    pnh_.getParam("initialUncertaintyVelXY", initialUncertaintyVelXY);
    pnh_.getParam("initialUncertaintyVelZ", initialUncertaintyVelZ);
    pnh_.getParam("initialUncertaintyOffsetXY", initialUncertaintyOffsetXY);
    pnh_.getParam("initialUncertaintyOffsetZ", initialUncertaintyOffsetZ);

    // Advertise publish topic
    string pub_topic{"target_tracker/pose"};
    pnh_.getParam("pub_topic", pub_topic);
    targetPub_ = nh_.advertise<PoseWithCovarianceStamped>(pub_topic, 10);

    string velPub_topic{"target_tracker/twist"};
    pnh_.getParam("velPub_topic", velPub_topic);
    targetVelPub_ = nh_.advertise<TwistWithCovarianceStamped>(velPub_topic, 10);



    // Advertise publish topic
    string offset_topic{"target_tracker/offset"};
    pnh_.getParam("offset_topic", offset_topic);
    offsetPub_ = nh_.advertise<PoseWithCovarianceStamped>(offset_topic, 10);


    ROS_INFO_STREAM("Publishing to " << targetPub_.getTopic());
    ROS_INFO_STREAM("Offset Publishing to " << offsetPub_.getTopic());

    // Time threshold
    pnh_.getParam("reset_time_threshold", time_threshold);

    // Wait for a valid stamp
    ROS_INFO("Waiting for valid stamp");
    ros::Time::waitForValid();
    ROS_INFO("Time received, proceeding");

    // Cache definition
    int cache_size{20};
    pnh_.getParam("cache_size", cache_size);
    ROS_ASSERT(cache_size > 0);
    state_cache_.set_cache_size((std::size_t) cache_size);

    // generate particles
    generateParticles();



    // Initialize the filter
    initializeFilter();
    initializeSubscribers();
}

void DistributedPF3D::generateParticles() {
    particles.resize(particle_num);
    weights.resize(particle_num);
    std::normal_distribution<double> dxy(initialParticlePosXY, sqrt(initialUncertaintyPosXY)),
                                     dz(initialParticlePosZ, sqrt(initialUncertaintyPosZ)),
                                     dvxy(initialParticleVel, sqrt(initialUncertaintyVelXY)),
                                     dvz(initialParticleVel, sqrt(initialUncertaintyVelZ));
                                    //  doxy(initialParticleOffset, sqrt(initialUncertaintyOffsetXY)),
                                    //  doz(initialParticleOffset,sqrt(initialUncertaintyOffsetZ));
    std::default_random_engine eng;
    for (int i = 0; i < particle_num; ++i) {
        Particle p(state_size);
        weights[i] = double(1.0 / particle_num);
        p.state << dxy(eng), dxy(eng), dz(eng), dvxy(eng), dvxy(eng), dvz(eng);//doxy(eng), doxy(eng), doz(eng);
        particles[i] = p;
    }
}
void DistributedPF3D::initializeSubscribers() {
    // Self and other robots info
    int robotID{0};
    int numRobots{0};
    pnh_.getParam("robotID", robotID);
    pnh_.getParam("numRobots", numRobots);

    // Pose subscriber
    std::string pose_topic{"pose"};
    pnh_.getParam("pose_topic", pose_topic);
    pose_sub_ = nh_.subscribe(pose_topic, 300, &DistributedPF3D::predictAndPublish, this);

    // Measurement subscribers
    string measurement_suffix_self{"/nonono"};
    string measurement_suffix{"/nonono"};
    pnh_.getParam("measurement_topic_suffix_self", measurement_suffix_self);
    pnh_.getParam("measurement_topic_suffix", measurement_suffix);

    selfcallbackhandler= unique_ptr<Callbackhandler>(new Callbackhandler(this,true,robotID));
    self_sub_ = nh_.subscribe(measurement_suffix_self, 50, &Callbackhandler::callback, selfcallbackhandler.get());
    ROS_INFO_STREAM("Registered self measurement subscriber for topic " << self_sub_.getTopic());

    for (int robot = 1; robot <= numRobots; robot++) {
        if (robot == robotID)
        continue;

        std::shared_ptr<Callbackhandler> cb(new Callbackhandler(this,false,robot));
        callbackhandlers.emplace_back(cb);
        const auto other_topic = "/machine_" + to_string(robot) + '/' + measurement_suffix;
        other_subs_.emplace_back(unique_ptr<ros::Subscriber>(
            new ros::Subscriber(nh_.subscribe(
                other_topic, 50, &Callbackhandler::callback,cb.get()))
        ));

        ROS_INFO_STREAM(
            "Registered other robot's measurements subscriber for topic " << other_subs_.back()->getTopic());
    }
}

void DistributedPF3D::initializeFilter() {
    // If there is a last element, grab its frame id; if not, use default world_frame
    std::string frame_id{world_frame};
    if (!state_cache_.empty())
        frame_id = state_cache_.back().frame_id;

    // Reset the cache
    state_cache_.clear();

    // Put an initial unknown estimate in the cache
    std_msgs::Header h;
    h.frame_id = frame_id;
    h.stamp = ros::Time::now();
    CacheElement first_element(h, state_size, true, 0);
    first_element.frame_id = frame_id;
    state_cache_.insert_ordered(first_element);

    ROS_INFO("The filter was (re)initialized");
}

void DistributedPF3D::measurementsCallback(const PoseWithCovarianceStamped& msg, const bool isSelf, const int robot) {
    if (detectBackwardsTimeJump()) {
        ROS_WARN("Backwardstimejump in cache - ignoring update");
        return;
    }

    if (state_cache_.empty()) {
        ROS_WARN("Cache is empty - ignoring update");
        return;
    }

    //    ROS_INFO("Measurement callback");
    // Create a new element for the cache
    CacheElement new_element(state_size, msg, isSelf, robot);

    // Insert this element into cache, which returns the iterator at insert position
    auto it = state_cache_.insert_ordered(new_element);

    // Check if failure to insert - this would be due to a very old message
    // Currently we decide to just alert the user, but this would be a good place to detect a jumping backwards and reset the filter
    if (it == state_cache_.end()) {
        ROS_WARN_STREAM(
            "Trying to insert a measurement that is too old! This is its stamp " << msg.header.stamp << std::endl
                                                                                << "Did you forget to reiniatilize the node after going back to the past e.g. stop and restart playback?");
        return;
    }

    // Rare, but may occur
    if(it == state_cache_.begin())
        ++it;

    // In a loop until we go through the whole cache, keep predicting and updating
    for (; it != state_cache_.end(); ++it) {
        if (!predict(*(it - 1), *it)) {
            ROS_WARN("Prediction step failed!");
            return;
        }
        if (it->measurements.size() > 0) {
            if (!update(*it)) {
                ROS_WARN("Rewind/Update failed!");
                return;
            }
        }
    }
}

MatrixXd DistributedPF3D::genTransformMatrix(double velocityDecayTo, double offsetDecayTo, double deltaT) {
    MatrixXd T(state_size, state_size);
    const double velocityDecayAlpha = pow(velocityDecayTo, 1.0 / velocityDecayTime);
    const double velocityDecayFactor = pow(velocityDecayAlpha, deltaT);
    const double velocityIntegralFactor = (velocityDecayFactor - 1.0) / log(velocityDecayAlpha);
    const double offsetDecayAlpha = pow(offsetDecayTo, 1.0 / offsetDecayTime);
    const double offsetDecayFactor = pow(offsetDecayAlpha, deltaT);
    T << 1.0, 0, 0, velocityIntegralFactor, 0, 0,
         0, 1.0, 0, 0, velocityIntegralFactor, 0, 
         0, 0, 1.0, 0, 0, velocityIntegralFactor,
         0, 0, 0, velocityDecayFactor, 0, 0, 
         0, 0, 0, 0, velocityDecayFactor, 0, 
         0, 0, 0, 0, 0, velocityDecayFactor;
    
    return T;
}

double DistributedPF3D::calculateWeight(VectorXd p, VectorXd m, double std) {
    VectorXd d = p - m;
    d = d * d;
    return exp(-(pow(d(0), 2)+pow(d(1), 2)+pow(d(2), 2))/2/std);
}

bool DistributedPF3D::predict(const CacheElement &in, CacheElement &out) {
    // Time past from one to next
    if (!out.stamp.isValid() || !in.stamp.isValid()) {
        ROS_WARN("One of the stamps is invalid, returning false from predict() without doing anything else");
        return false;
    }

    const double deltaT = out.stamp.toSec() - in.stamp.toSec();

    if (deltaT > time_threshold) {
        ROS_WARN_STREAM("It's been a long time since there was an update (" << deltaT
                                                                            << " seconds). Resetting the filter to be safe... position(0,0,0) and high uncertainty");
        initializeFilter();
        return false;
    }

    const static double offsetDecayTo = 0.1;
    const static double velocityDecayTo = 0.1;
    
    MatrixXd T = genTransformMatrix(velocityDecayTo, offsetDecayTo, deltaT);
    VectorXd s = VectorXd::Zero(state_size);
    for (int i = 0; i < particle_num; ++i){
        particles[i].state = T * particles[i].state;
        s += particles[i].state * weights[i];
    }
    out.state = s;
    ROS_INFO_STREAM("predict POSE="<< s(0) <<s(1) << s(2));
    ROS_INFO_STREAM("predict VELO="<<s(3) << s(4) << s(5));
    return true;
}

bool DistributedPF3D::update(CacheElement &elem) {

    //    ROS_INFO("Update");

    if (elem.measurements.empty()) {
        ROS_WARN("Tried to perform update step with no measurements in element. Returning without doing anything");
        return false;
    }

    // Find out closest measurement to current state estimate and use that one
    int closest_idx = -1;
    double min_error{DBL_MAX};
    for (size_t i = 0; i < elem.measurements.size(); ++i) {

        const auto measurement = elem.measurements[i];

        double difference[3]{measurement->pose.position.x - elem.state(0),
                            measurement->pose.position.y - elem.state(1),
                            measurement->pose.position.z - elem.state(2)};

        double sqr_error{
            sqrt(difference[0] * difference[0] + difference[1] * difference[1] + difference[2] * difference[2])};

        if (sqr_error < min_error) {
            min_error = sqr_error;
            closest_idx = i;
        }
    }
    
    if (closest_idx < 0 || closest_idx > (int) elem.measurements.size()) {
        ROS_ERROR("Something went wrong, couldn't didn't find the closest measurement");
        return false;
    }

    const auto closest_measurement = elem.measurements[closest_idx];

    VectorXd e_measurement((int) measurement_state_size);
    VectorXd state_u = VectorXd::Zero(state_size);

    // we aren't really measuring the offset, we can only measure the difference between observed and predicted target, which should be offset corrected already

    double measured_offset_x = elem.state(6) - (closest_measurement->pose.position.x - elem.state(0));
    double measured_offset_y = elem.state(7) - (closest_measurement->pose.position.y - elem.state(1));
    double measured_offset_z = elem.state(8) - (closest_measurement->pose.position.z - elem.state(2));

    e_measurement
        << closest_measurement->pose.position.x, closest_measurement->pose.position.y, closest_measurement->pose.position.z, measured_offset_x, measured_offset_y, measured_offset_z;
    ROS_INFO_STREAM("measured"<< closest_measurement->pose.position.x <<"  " << closest_measurement->pose.position.y << "  " << closest_measurement->pose.position.z);
    double w = 0;
    for (int i = 0; i < particle_num; ++i){
        VectorXd p((int) measurement_state_size);
        p << particles[i].state(0), particles[i].state(1), particles[i].state(2), particles[i].state(6), particles[i].state(7), particles[i].state(8);
        weights[i] = calculateWeight(p, e_measurement, 1);
        w += weights[i];
    }

    for (int i = 0; i < particle_num; ++i) {
        weights[i] /= w;
        state_u += weights[i] * particles[i].state;
    }
    elem.state = state_u;
    ROS_INFO_STREAM("calculated"<< state_u(0) << "  " << state_u(1) << "  " << state_u(2));
    p_eff = 0;
    
    for(int i = 0; i < particle_num; ++i) p_eff += pow(weights[i], 2);
    if (1/p_eff < particle_num *0.1) {
        MatrixXd cov = MatrixXd::Zero(state_size, state_size);
        for (int i = 0; i < particle_num; ++i)
            cov += weights[i] * (particles[i].state - state_u) * (particles[i].state - state_u).transpose();
        LLT<MatrixXd> llt(cov);
        MatrixXd L = llt.matrixL();
        std::vector<Particle> r_particle;
        r_particle.resize(particle_num);
        std::default_random_engine eng;
        std::discrete_distribution<int> discreteDistribution(weights.begin(), weights.end());
        for (int i = 0; i < this->particle_num; ++i)
            r_particle[i] = this->particles[discreteDistribution(eng)];

        double A = 0.92513;
        double h = A * 0.45249;
        std::normal_distribution<double> dxy{0, 10};
        std::normal_distribution<double> dz{0, 6};
        std::normal_distribution<double> dv{0, 3};
        std::normal_distribution<double> dvz{0, 0.5};
        for (int i = 0; i < particle_num; ++i) {
            VectorXd v(state_size);
            v << dxy(eng), dxy(eng), dz(eng), dv(eng), dv(eng), dvz(eng);
            v = L * v;
            r_particle[i].state += h * v;
            
        }
        particles = r_particle;
    }
    return true;
}

void DistributedPF3D::predictAndPublish(const uav_msgs::uav_poseConstPtr &pose) {
  if (state_cache_.empty())
    return;

//    ROS_INFO_STREAM(state_cache_);
  // Always self robot because predict is only called for self poses
  CacheElement tmp_element(pose->header, state_size, true,0);

  const auto last = state_cache_.back();
  if (!predict(last, tmp_element))
    return;

//    ROS_INFO("Predict and Publish");
  publishStateAndCov(tmp_element);
}

void DistributedPF3D::publishStateAndCov(const CacheElement &elem) {

  msg_.header.frame_id = elem.frame_id;
  msg_.header.stamp = elem.stamp;

  msg_.pose.pose.position.x = elem.state[0];
  msg_.pose.pose.position.y = elem.state[1];
  msg_.pose.pose.position.z = elem.state[2];
  msg_.pose.covariance[0 * 6 + 0] = elem.cov(0 * 9 + 0);
  msg_.pose.covariance[0 * 6 + 1] = elem.cov(0 * 9 + 1);
  msg_.pose.covariance[0 * 6 + 2] = elem.cov(0 * 9 + 2);
  msg_.pose.covariance[1 * 6 + 0] = elem.cov(1 * 9 + 0);
  msg_.pose.covariance[1 * 6 + 1] = elem.cov(1 * 9 + 1);
  msg_.pose.covariance[1 * 6 + 2] = elem.cov(1 * 9 + 2);
  msg_.pose.covariance[2 * 6 + 0] = elem.cov(2 * 9 + 0);
  msg_.pose.covariance[2 * 6 + 1] = elem.cov(2 * 9 + 1);
  msg_.pose.covariance[2 * 6 + 2] = elem.cov(2 * 9 + 2);
  ROS_INFO_STREAM("PUBLISHED"<<elem.state[0]<<"  "<<elem.state[1]<<"  "<<elem.state[2]);
  msg_.pose.pose.orientation.w = 1.0;

  targetPub_.publish(msg_);

  velMsg_.twist.twist.linear.x = elem.state[3];
  velMsg_.twist.twist.linear.y = elem.state[4];
  velMsg_.twist.twist.linear.z = elem.state[5];
  targetVelPub_.publish(velMsg_);

///*
  msg_.pose.pose.position.x = elem.state[6];
  msg_.pose.pose.position.y = elem.state[7];
  msg_.pose.pose.position.z = elem.state[8];
  msg_.pose.covariance[0 * 6 + 0] = elem.cov(6 * 9 + 6);
  msg_.pose.covariance[0 * 6 + 1] = elem.cov(6 * 9 + 7);
  msg_.pose.covariance[0 * 6 + 2] = elem.cov(6 * 9 + 8);
  msg_.pose.covariance[1 * 6 + 0] = elem.cov(7 * 9 + 6);
  msg_.pose.covariance[1 * 6 + 1] = elem.cov(7 * 9 + 7);
  msg_.pose.covariance[1 * 6 + 2] = elem.cov(7 * 9 + 8);
  msg_.pose.covariance[2 * 6 + 0] = elem.cov(8 * 9 + 6);
  msg_.pose.covariance[2 * 6 + 1] = elem.cov(8 * 9 + 7);
  msg_.pose.covariance[2 * 6 + 2] = elem.cov(8 * 9 + 8);


  offsetPub_.publish(msg_);

  // Debug - output full state
//    ROS_INFO_STREAM("Full state at time " << ros::Time::now() << std::endl << elem.state << std::endl << "And covariance " << std::endl << elem.cov);
}

void DistributedPF3D::dynamicReconfigureCallback(ParticleFilterParamsConfig &config,
                                                 uint32_t level) {

  ROS_INFO("Received reconfigure request");
  noisePosXVar = config.noisePosXVar;
  noiseVelXVar = config.noiseVelXVar;
  noiseOffXVar = config.noiseOffXVar;

  noisePosYVar = config.noisePosYVar;
  noiseVelYVar = config.noiseVelYVar;
  noiseOffYVar = config.noiseOffYVar;

  noisePosZVar = config.noisePosZVar;
  noiseVelZVar = config.noiseVelZVar;
  noiseOffZVar = config.noiseOffZVar;

  velocityDecayTime = config.velocityDecayTime;
  offsetDecayTime = config.offsetDecayTime;

  // Reinitialize matrices
//   initializeStaticMatrices();
//   ROS_INFO_STREAM("Process noise matrix" << std::endl << R);

  // Reinitialize filter
  initializeFilter();
}

bool DistributedPF3D::detectBackwardsTimeJump() {
  // Do not detect if not using sim time
  const static bool using_sim_time = ros::Time::isSimTime();
  if (!using_sim_time)
    return false;

  static auto time = ros::Time::now();

  if (ros::Time::now() < time) {
    // Jump backwards detected, reset interface
    ROS_WARN("Backwards jump in time detected, performing reset");
    initializeFilter();
    time = ros::Time::now();
    return true;
  }
  time = ros::Time::now();
  return false;
}
}
