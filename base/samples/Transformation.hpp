#ifndef BASE_SAMPLES_TRANSFORMATION_HPP
#define BASE_SAMPLES_TRANSFORMATION_HPP

#include <string>
#include <base/Time.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace base { namespace samples {

/**
 * Represents a transformation in 3D of a local coordinate frame inside a global coordinate frame.
 * A frame ID shall be globally unique and corresponds to a coordinate system.
 * Following the [Rock's conventions](http://rock.opendfki.de/wiki/WikiStart/Standards)
 * coordinate systems are right handed with X forward, Y left and Z up.
 *
 * The transformation \f$ ^{global}_{local}T \f$ in this structure transforms a vector \f$ v_{local} \f$
 * from the local into the global frame:
 * \f$ v_{global} = ^{global}_{local}T \cdot v_{local} \f$
 * In other words the transformation is the pose of the local frame expressed in the global frame.
 * Example of a transformation chain:
 * \f$ ^{world}_{sensor}T = ^{world}_{body}T \cdot ^{body}_{sensor}T \f$
 */
class Transformation
{
public:
    /** Reference timestamp of the transformation sample */
    base::Time time;

    /** ID of the global reference frame */
    std::string global_frame_id;

    /** ID of the local reference frame */
    std::string local_frame_id;

    /** Transformation expressed in the global frame */
    base::TransformWithCovariance transform;

public:
    Transformation() {}
    Transformation(const base::TransformWithCovariance& transform) :
        transform(transform.translation, transform.orientation, transform.cov) {}
    Transformation(const base::samples::RigidBodyState& rbs) : time(rbs.time),
        global_frame_id(rbs.targetFrame), local_frame_id(rbs.sourceFrame), transform(rbs.position, rbs.orientation)
    {
        transform.cov << rbs.cov_position, Eigen::Matrix3d::Zero(),
                         Eigen::Matrix3d::Zero(), rbs.cov_orientation;
    }

    void setTransform(const base::TransformWithCovariance& transform)
    {
        this->transform = transform;
    }

    void setTransform(const Eigen::Affine3d& transform)
    {
        this->transform.setTransform(transform);
    }

    const base::TransformWithCovariance& getTransformWithCovariance() const
    {
        return transform;
    }

    Eigen::Affine3d getTransform() const
    {
        return transform.getTransform();
    }

    base::samples::RigidBodyState toRigidBodyState() const
    {
        base::samples::RigidBodyState rbs;
        rbs.time = time;
        rbs.targetFrame = global_frame_id;
        rbs.sourceFrame = local_frame_id;
        rbs.position = transform.translation;
        rbs.orientation = transform.orientation;
        if(transform.hasValidCovariance())
        {
            rbs.cov_position = transform.getTranslationCov();
            rbs.cov_orientation = transform.getOrientationCov();
        }
        return rbs;
    }

};

}}

#endif