#include "suction_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/World.hh>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

SuctionPlugin::SuctionPlugin()
{
    std::cout << "======= SuctionPlugin constructed =======" << std::endl;
}

void SuctionPlugin::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
    this->modelEntity = _entity;
    std::cout << "======= Configure started. Entity ID: " << _entity << " =======" << std::endl;

    // Check if entity is valid and has Name component
    auto name = _ecm.Component<components::Name>(_entity);
    if (name)
    {
        this->modelName = name->Data();
        std::cout << "Model name attached to plugin: " << this->modelName << std::endl;
    }
    else
    {
        std::cerr << "ERROR: Entity does not have Name component!" << std::endl;
    }

    // Discover world name
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &, const components::World *, const components::Name *_name) -> bool
        {
            this->worldName = _name->Data();
            std::cout << "World name: " << this->worldName << std::endl;
            return false; // Only need first world
        });

    // Read suction radius from SDF
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction radius set to: " << this->suctionRadius << std::endl;
    }
    else
    {
        std::cout << "Using default suction radius: " << this->suctionRadius << std::endl;
    }

    // Read filter name from SDF
    if (_sdf->HasElement("filter_name"))
    {
        this->filterName = _sdf->Get<std::string>("filter_name");
        std::cout << "Filter name set to: " << this->filterName << std::endl;
    }
    else
    {
        std::cout << "Using default filter name: " << this->filterName << std::endl;
    }

    // Read parent link name from SDF
    if (_sdf->HasElement("link_name"))
    {
        this->parentLinkName = _sdf->Get<std::string>("link_name");
        std::cout << "Parent link name set to: " << this->parentLinkName << std::endl;
    }

    // Read use_suction_radius from SDF
    if (_sdf->HasElement("use_suction_radius"))
    {
        this->useSuctionRadius = _sdf->Get<bool>("use_suction_radius");
        std::cout << "Use suction radius: " << (this->useSuctionRadius ? "TRUE" : "FALSE") << std::endl;
    }
    
    if (_sdf->HasElement("suction_force"))
    {
        this->suctionForce = _sdf->Get<double>("suction_force");
        std::cout << "Suction Force: " << this->suctionForce << std::endl;
    }
    
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction Radius: " << this->suctionRadius << std::endl;
    }

    if (_sdf->HasElement("suction_topic"))
    {
        this->suctionTopic = _sdf->Get<std::string>("suction_topic");
        std::cout << "Suction Topic: " << this->suctionTopic << std::endl;
    }

    // Read optional contact_topic override from SDF
    if (_sdf->HasElement("contact_topic"))
    {
        this->contactTopic = _sdf->Get<std::string>("contact_topic");
        std::cout << "Contact Topic (from SDF): " << this->contactTopic << std::endl;
    }

    // Create and advertise suction activation topic
    this->publisher = this->node.Advertise<msgs::Boolean>(this->suctionTopic);
    if (!this->publisher)
    {
        std::cerr << "ERROR: Failed to advertise topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Topic advertised successfully: " << this->suctionTopic << std::endl;
    }

    // Subscribe to suction activation topic
    if (!this->node.Subscribe(this->suctionTopic,
                              &SuctionPlugin::OnSuctionActivate,
                              this))
    {
        std::cerr << "ERROR: Failed to subscribe to topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Successfully subscribed to topic: " << this->suctionTopic << std::endl;
    }

    // Build and subscribe to the contact topic if in contact mode
    if (!this->useSuctionRadius && !this->parentLinkName.empty())
    {
        // If no explicit contact_topic, build the default Gazebo Harmonic path
        if (this->contactTopic.empty() && !this->worldName.empty() && !this->modelName.empty())
        {
            this->contactTopic = "/world/" + this->worldName
                + "/model/" + this->modelName
                + "/link/" + this->parentLinkName
                + "/sensor/suction_contact/contact";
        }

        if (!this->contactTopic.empty())
        {
            if (this->node.Subscribe(this->contactTopic,
                                     &SuctionPlugin::OnContactData,
                                     this))
            {
                this->contactSubscribed = true;
                std::cout << "[SuctionPlugin] Subscribed to contact topic: " << this->contactTopic << std::endl;
            }
            else
            {
                std::cerr << "[SuctionPlugin] ERROR: Failed to subscribe to contact topic: " << this->contactTopic << std::endl;
            }
        }
        else
        {
            std::cerr << "[SuctionPlugin] WARNING: Could not build contact topic path. "
                      << "worldName='" << this->worldName << "' modelName='" << this->modelName << "'" << std::endl;
        }
    }

    std::cout << "======= Configure completed =======" << std::endl;
}

void SuctionPlugin::OnContactData(const msgs::Contacts &_msg)
{
    std::lock_guard<std::mutex> lock(this->contactMutex);
    this->latestContactMsg = _msg;
    this->hasNewContactData = true;
}

void SuctionPlugin::PreUpdate(const UpdateInfo &_info,
                              EntityComponentManager &_ecm)
{
    // 1. Logic for Suction (Sensing/Actuation)
    if (this->suctionActive && this->attachedEntity == kNullEntity)
    {
        if (this->useSuctionRadius)
        {
            this->FindTargetRadius(_ecm);
        }
        // Contact mode: process transport messages in PreUpdate (where we can modify ECM)
        else if (this->contactSubscribed)
        {
            this->ProcessContactMessages(_ecm);
        }
    }

    // 2. Handle Attachment (Joint Creation)
    if (this->targetEntityToAttach != kNullEntity)
    {
        if (this->attachedEntity == kNullEntity) // Double check
        {
             // Find gripper link
             Entity gripperLink = kNullEntity;
             if (!this->parentLinkName.empty())
             {
                auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
                for (const auto &link : links) {
                    auto nameComp = _ecm.Component<components::Name>(link);
                    if (nameComp && nameComp->Data() == this->parentLinkName) { gripperLink = link; break; }
                }
             }

             // Find target link
             Entity targetLink = kNullEntity;
             auto targetLinks = _ecm.ChildrenByComponents(this->targetEntityToAttach, components::Link());
             if (!targetLinks.empty()) targetLink = targetLinks[0];

             if (gripperLink != kNullEntity && targetLink != kNullEntity)
             {
                 this->jointEntity = _ecm.CreateEntity();
                 components::DetachableJointInfo jointInfo;
                 jointInfo.parentLink = gripperLink;
                 jointInfo.childLink = targetLink;
                 jointInfo.jointType = "fixed";
                 _ecm.CreateComponent(this->jointEntity, components::DetachableJoint(jointInfo));
                 _ecm.CreateComponent(this->jointEntity, components::Name("suction_joint"));
                 
                 this->attachedEntity = this->targetEntityToAttach;
                 std::cout << "SUCCESS: Attached to " << this->attachedEntity << std::endl;
             }
             else
             {
                 std::cerr << "ERROR: Failed to find links for attachment in PreUpdate."
                           << " gripperLink=" << gripperLink << " targetLink=" << targetLink << std::endl;
             }
        }
        this->targetEntityToAttach = kNullEntity; // Reset
    }

    // 3. Handle Detachment
    if (!this->suctionActive && this->attachedEntity != kNullEntity)
    {
        this->Detach(_ecm);
    }
}

void SuctionPlugin::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
    // PostUpdate no longer used for contact detection.
    // Contact data is now received via gz transport subscription
    // and processed in PreUpdate.
}

void SuctionPlugin::ProcessContactMessages(const EntityComponentManager &_ecm)
{
    msgs::Contacts contactMsg;
    {
        std::lock_guard<std::mutex> lock(this->contactMutex);
        if (!this->hasNewContactData)
            return;
        contactMsg = this->latestContactMsg;
        this->hasNewContactData = false;
    }

    static int dbgCount = 0;
    bool dbg = (++dbgCount % 500 == 1);

    if (dbg) std::cout << "[SuctionContact] Processing " << contactMsg.contact_size() << " contacts" << std::endl;

    for (const auto &contact : contactMsg.contact())
    {
        Entity col1 = contact.collision1().id();
        Entity col2 = contact.collision2().id();

        // Traverse collision -> link -> model
        auto GetModelFromCollision = [&](Entity _colEntity) -> Entity {
            auto linkEntity = _ecm.ParentEntity(_colEntity);
            if (linkEntity == kNullEntity) return kNullEntity;
            return _ecm.ParentEntity(linkEntity);
        };

        Entity model1 = GetModelFromCollision(col1);
        Entity model2 = GetModelFromCollision(col2);

        if (dbg) std::cout << "[SuctionContact] col1=" << col1 << " col2=" << col2
                           << " model1=" << model1 << " model2=" << model2
                           << " myModel=" << this->modelEntity << std::endl;

        Entity potentialTarget = kNullEntity;
        if (model1 == this->modelEntity && model2 != kNullEntity) potentialTarget = model2;
        else if (model2 == this->modelEntity && model1 != kNullEntity) potentialTarget = model1;

        if (potentialTarget != kNullEntity)
        {
            auto nameComp = _ecm.Component<components::Name>(potentialTarget);
            std::cout << "[SuctionContact] potentialTarget=" << potentialTarget
                      << " name='" << (nameComp ? nameComp->Data() : "NULL")
                      << "' filterName='" << this->filterName << "'" << std::endl;
            if (nameComp && nameComp->Data().find(this->filterName) != std::string::npos)
            {
                this->targetEntityToAttach = potentialTarget;
                std::cout << "[SuctionContact] >>> MATCH! Scheduling attachment to " << potentialTarget << std::endl;
                return;
            }
        }
        else if (dbg)
        {
            std::cout << "[SuctionContact] No potentialTarget: neither model matches myModel" << std::endl;
        }
    }
}

void SuctionPlugin::FindTargetRadius(EntityComponentManager &_ecm)
{
    // Find the gripper link
    Entity gripperLink = kNullEntity;
    if (!this->parentLinkName.empty())
    {
        auto links = _ecm.ChildrenByComponents(this->modelEntity, components::Link());
        for (const auto &link : links)
        {
            auto nameComp = _ecm.Component<components::Name>(link);
            if (nameComp && nameComp->Data() == this->parentLinkName) { gripperLink = link; break; }
        }
    }

    if (gripperLink == kNullEntity) return;

    // Calculate Gripper Link World Pose using proper kinematic chain traversal
    math::Pose3d gripperWorldPose = worldPose(gripperLink, _ecm);

    Entity nearestEntity = kNullEntity;
    double minDistance = std::numeric_limits<double>::max();

    // Iterate all models to find candidates
    _ecm.Each<components::Name, components::Pose, components::Model>(
        [&](const Entity &_entity,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Model *) -> bool
        {
            if (_entity == this->modelEntity) return true;
            if (!this->filterName.empty() && _name->Data().find(this->filterName) == std::string::npos) return true;

            double distance = this->CalculateDistance(gripperWorldPose, _pose->Data());
            
            if (distance <= this->suctionRadius)
            {
                // --- APPLY SUCTION FORCE ---
                auto targetLinks = _ecm.ChildrenByComponents(_entity, components::Link());
                if (!targetLinks.empty())
                {
                    Entity targetLink = targetLinks[0];
                    
                    // Calculate vector from target to gripper
                    math::Vector3d direction = gripperWorldPose.Pos() - _pose->Data().Pos();
                    direction.Normalize();
                    
                    math::Vector3d force = direction * this->suctionForce;
                    
                    // Add External World Wrench
                    auto wrenchComp = _ecm.Component<components::ExternalWorldWrenchCmd>(targetLink);
                    if (!wrenchComp)
                    {
                        components::ExternalWorldWrenchCmd wrenchCmd;
                        msgs::Wrench wrenchMsg;
                        msgs::Set(wrenchMsg.mutable_force(), force);
                        wrenchCmd.Data() = wrenchMsg;
                        _ecm.CreateComponent(targetLink, wrenchCmd);
                    }
                    else
                    {
                        msgs::Set(wrenchComp->Data().mutable_force(), force);
                    }
                }

                // Check for attachment
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestEntity = _entity;
                }
            }
            return true;
        });

    // If close enough (e.g. < 0.02m), schedule attachment
    if (nearestEntity != kNullEntity && minDistance < 0.02) 
    {
        this->targetEntityToAttach = nearestEntity;
    }
}

void SuctionPlugin::Detach(EntityComponentManager &_ecm)
{
    if (this->jointEntity != kNullEntity)
    {
        std::cout << "Detaching entity: " << this->attachedEntity << std::endl;
        _ecm.RequestRemoveEntity(this->jointEntity);
        this->jointEntity = kNullEntity;
        this->attachedEntity = kNullEntity;
    }
}

void SuctionPlugin::OnSuctionActivate(const msgs::Boolean &_msg)
{
    bool newState = _msg.data();
    std::cout << "======= ALERT: Received suction command: " << (newState ? "ACTIVATE" : "DEACTIVATE") << " =======" << std::endl;
    this->suctionActive = newState;
}

double SuctionPlugin::CalculateDistance(const math::Pose3d &_pose1,
                                        const math::Pose3d &_pose2)
{
    return _pose1.Pos().Distance(_pose2.Pos());
}

GZ_ADD_PLUGIN(SuctionPlugin,
              gz::sim::System,
              SuctionPlugin::ISystemConfigure,
              SuctionPlugin::ISystemPreUpdate,
              SuctionPlugin::ISystemPostUpdate)