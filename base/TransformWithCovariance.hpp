#ifndef __BASE_TRANSFORM_WITH_COVARIANCE_HPP__
#define __BASE_TRANSFORM_WITH_COVARIANCE_HPP__

#include <iomanip> // std::setprecision

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Float.hpp>
#include <base/Pose.hpp>
#include <base/Eigen.hpp>

namespace base {

    /** 
     * Class which represents a 3D Transformation with associated uncertainty information.
     *
     * The uncertainty is represented as a 6x6 matrix, which is the covariance
     * matrix of the [r t] representation of the error. Here r is the rotation orientation
     * part expressed as a scaled axis of orientation, and t the translational
     * component.
     *
     * The uncertainty information is optional. The hasValidCovariance() method can
     * be used to see if uncertainty information is associated with the class.
     */
    class TransformWithCovariance
    {

    public:
	    typedef base::Matrix6d Covariance;

    public:
        /** The transformation is represented 6D vector [translation orientation]
        * Here orientation is the rotational part expressed as a quaternion
        * orientation, and t the translational component.
        */
        base::Position translation;

        base::Quaterniond orientation;

        /** The uncertainty is represented as a 6x6 matrix, which is the covariance
         * matrix of the [translation orientation] representation of the error.
         */
        Covariance cov;

    public:
      
        TransformWithCovariance() : translation(base::Position::Zero()), 
            orientation(base::Quaterniond::Identity()) {this->invalidateCovariance();}
      
        explicit TransformWithCovariance( const base::Affine3d& trans)
            {this->setTransform(trans); this->invalidateCovariance();};

        TransformWithCovariance( const base::Affine3d& trans, const Covariance& cov )
            {this->setTransform(trans); this->cov = cov;};

        TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation) :
            translation(translation), orientation(orientation) {this->invalidateCovariance();};

        TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation, const Covariance& cov ) :
            translation(translation), orientation(orientation), cov(cov){};

        static TransformWithCovariance Identity()
        {
            return TransformWithCovariance();
        };
	
	
        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);

        /** performs a composition of this transform with the transform given.
         * The result is another transform with result = this * trans
         */
        TransformWithCovariance composition( const TransformWithCovariance& trans ) const
        {
            return this->operator*( trans );
        };

        /** performs an inverse composition of two transformations.
         * The result is such that result * trans = this. Note that this is different from
         * calling result = this * inv(trans), in the way the uncertainties are handled.
         */
        TransformWithCovariance compositionInv( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &tf(*this);
            const TransformWithCovariance &t1(trans);
            base::Position p2(tf.translation + (tf.orientation * t1.inverse().translation));
            Eigen::Quaterniond q2( tf.orientation * t1.orientation.inverse());

            // short path if there is no uncertainty
            if( !t1.hasValidCovariance() && !tf.hasValidCovariance() )
                return TransformWithCovariance(p2, q2);

            // convert the orientations of the respective transforms into quaternions
            // in order to inverse the covariances, we need to get both the t1 and t2=[r2 p2] transformations
            // based on the composition tf = t2 * t1
            Eigen::Quaterniond q1( t1.orientation );
            Eigen::Quaterniond q( q2 * q1 );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            Eigen::Matrix<double,6,6> J1;
            J1 << q2.toRotationMatrix(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

            Eigen::Matrix<double,6,6> J2;
            J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, t1.translation),
            Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

            cov = J2.inverse() * ( tf.getCovariance() - J1 * t1.getCovariance() * J1.transpose() ) * J2.transpose().inverse();

            // and return the resulting uncertainty transform
            return TransformWithCovariance( p2, q2, cov );
        };

        /** Same as compositionInv, just that the result is such that trans * result = this.
         * Note that this is different from calling result = inv(trans) * this, 
         * in the way the uncertainties are handled.
         */
        TransformWithCovariance preCompositionInv( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &tf(*this);
            const TransformWithCovariance &t2(trans);
            base::Position p1(t2.inverse().translation + (t2.orientation.inverse() * tf.translation));
            Eigen::Quaterniond q1(t2.orientation.inverse() * tf.orientation);

            // short path if there is no uncertainty 
            if( !t2.hasValidCovariance() && !tf.hasValidCovariance() )
                return TransformWithCovariance( p1, q1 );

            // convert the orientations of the respective transforms into quaternions
            // in order to inverse the covariances, we need to get both the t1=[p1 q1] and t2 transformations
            // based on the composition tf = t2 * t1
            Eigen::Quaterniond q2(t2.orientation);
            Eigen::Quaterniond q( q2 * q1 );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            Eigen::Matrix<double,6,6> J1;
            J1 << t2.getTransform().linear(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

            Eigen::Matrix<double,6,6> J2;
            J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, p1),
            Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

            cov = J1.inverse() * ( tf.getCovariance() - J2 * t2.getCovariance() * J2.transpose() ) * J1.transpose().inverse();

            // and return the resulting uncertainty transform
            return TransformWithCovariance( p1, q1, cov );
        };

        /** alias for the composition of two transforms
         */
        TransformWithCovariance operator*( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &t2(*this);
            const TransformWithCovariance &t1(trans);

            const base::Quaterniond t(t2.orientation * t1.orientation);
            const base::Position p(t2.translation + (t2.orientation * t1.translation));

            // short path if there is no uncertainty 
            if( !t1.hasValidCovariance() && !t2.hasValidCovariance() )
            {
                return TransformWithCovariance(p, t);
            }

            // convert the orientations of the respective transforms into quaternions
            const Eigen::Quaterniond q1( t1.orientation ), q2( t2.orientation );
            const Eigen::Quaterniond q( t2.orientation * t1.orientation );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            // calculate the Jacobians (this is what all the above functions are for)
            // and add to the resulting covariance
            if( t1.hasValidCovariance() )
            {
                Eigen::Matrix<double,6,6> J1;
                J1 << t2.getTransform().linear(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

                cov += J1*t1.getCovariance()*J1.transpose();
            }

            if( t2.hasValidCovariance() )
            {
                Eigen::Matrix<double,6,6> J2;
                J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, t1.translation),
                Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

                cov += J2*t2.getCovariance()*J2.transpose();
            }

            // and return the resulting uncertainty transform
            return TransformWithCovariance(p, t, cov);
        };
	
        TransformWithCovariance inverse() const
        {
            // short path if there is no uncertainty
            if( !hasValidCovariance() )
                return TransformWithCovariance(static_cast<base::Position>(-(this->orientation.inverse() * this->translation)), this->orientation.inverse());

            Eigen::Quaterniond q(this->orientation);
            Eigen::Vector3d t(this->translation);
            Eigen::Matrix<double,6,6> J;
            J << q.toRotationMatrix().transpose(), drx_by_dr( q.inverse(), t ),
            Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

            return TransformWithCovariance(static_cast<base::Position>(-(this->orientation.inverse() * this->translation)),
                                        this->orientation.inverse(),
                                        J*this->getCovariance()*J.transpose());
        };

        const Covariance& getCovariance() const { return this->cov; }
        void setCovariance( const Covariance& cov ) { this->cov = cov; }

        const base::Matrix3d getTranslationCov() const { return this->cov.topLeftCorner<3,3>(); }
        void setTranslationCov(const base::Matrix3d& cov) { this->cov.topLeftCorner<3,3>() = cov; }

        const base::Matrix3d getOrientationCov() const { return this->cov.bottomRightCorner<3,3>(); }
        void setOrientationCov(const base::Matrix3d& cov) { this->cov.bottomRightCorner<3,3>() = cov; }

        const base::Affine3d getTransform() const
        {
            base::Affine3d trans (this->orientation);
            trans.translation() = this->translation;
            return trans;
        }
        void setTransform( const base::Affine3d& trans )
        {
            this->translation = trans.translation();
            this->orientation = base::Quaterniond(trans.rotation());
        }

        const base::Orientation getOrientation() const
        {
            return base::Orientation(this->orientation);
        }

        void setOrientation(const base::Orientation & q)
        {
            this->orientation = base::Quaterniond(q);
        }

        bool hasValidTransform() const
        {
            return !translation.hasNaN() && !orientation.coeffs().hasNaN();
        }

        void invalidateTransform()
        {
            translation = base::Position::Ones() * base::NaN<double>();
            orientation.coeffs() = base::Vector4d::Ones() * base::NaN<double>();
        }

        /** @warning This method is computationally expensive. Use with care! */
        bool hasValidCovariance() const { return !cov.hasNaN(); }
        void invalidateCovariance()
        {
            cov = Covariance::Ones() * base::NaN<double>();
        }

    protected:
        // The uncertainty transformations are implemented according to: 
        // Pennec X, Thirion JP. A framework for uncertainty and validation of 3-D
        // registration methods based on points and frames. International Journal of
        // Computer Vion. 1997;25(3):203–229. Available at:
        // http://www.springerlink.com/index/JJ25N2Q23T402682.pdf.

        static Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r )
        {
            double theta = r.norm();
            if( fabs(theta) > 1e-5 )
                return Eigen::Quaterniond( base::AngleAxisd( theta, r/theta ) );
            else
                return Eigen::Quaterniond::Identity();
        }

        static Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q )
        {
            base::AngleAxisd aa( q );
            return aa.axis() * aa.angle();
        }

        static inline double sign( double v )
        {
            return v > 0.0 ? 1.0 : -1.0;
        }

        static Eigen::Matrix<double,3,3> skew_symmetric( const Eigen::Vector3d& r )
        {
            Eigen::Matrix3d res;
            res << 0, -r.z(), r.y(),
            r.z(), 0, -r.x(),
            -r.y(), r.x(), 0;
            return res;
        }

        static Eigen::Matrix<double,4,3> dq_by_dr( const Eigen::Quaterniond& q )
        {
            const Eigen::Vector3d r( q_to_r( q ) );

            const double theta = r.norm();
            const double kappa = 0.5 - theta*theta / 48.0; // approx. see Paper 
            const double lambda = 1.0/24.0*(1.0-theta*theta/40.0); // approx.
            Eigen::Matrix<double,4,3> res;
            res << - q.vec().transpose()/2.0,
            kappa * Eigen::Matrix3d::Identity() - lambda * r * r.transpose(); 	

            return res;
        }

        static Eigen::Matrix<double,3,4> dr_by_dq( const Eigen::Quaterniond& q )
        {
            const Eigen::Vector3d r( q_to_r( q ) );
            const double mu = q.vec().norm();
            const double tau = 2.0 * sign( q.w() ) * ( 1.0 + mu*mu/6.0 ); // approx
            const double nu = -2.0 * sign( q.w() ) * ( 2.0/3.0 + mu*mu/5.0 ); // approx

            Eigen::Matrix<double,3,4> res;
            res << -2*q.vec(), tau * Eigen::Matrix3d::Identity() + nu * q.vec() * q.vec().transpose();

            return res;
        }

        static Eigen::Matrix<double,4,4> dq2q1_by_dq1( const Eigen::Quaterniond& q2 )
        {
            Eigen::Matrix<double,4,4> res;
            res << 0, -q2.vec().transpose(),
            q2.vec(), skew_symmetric( q2.vec() );
            return Eigen::Matrix<double,4,4>::Identity() * q2.w() + res;
        }

        static Eigen::Matrix<double,4,4> dq2q1_by_dq2( const Eigen::Quaterniond& q1 )
        {
            Eigen::Matrix<double,4,4> res;
            res << 0, -q1.vec().transpose(),
            q1.vec(), -skew_symmetric( q1.vec() );
            return Eigen::Matrix<double,4,4>::Identity() * q1.w() + res;
        }

        static Eigen::Matrix<double,3,3> dr2r1_by_r1( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
        {
            return Eigen::Matrix3d(
                dr_by_dq( q )
                * dq2q1_by_dq1( q2 )
                * dq_by_dr( q1 ) );
        }

        static Eigen::Matrix<double,3,3> dr2r1_by_r2( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
        {
            return Eigen::Matrix3d(
                dr_by_dq( q )
                * dq2q1_by_dq2( q1 )
                * dq_by_dr( q2 ) );
        }

        static Eigen::Matrix<double,3,3> drx_by_dr( const Eigen::Quaterniond& q, const Eigen::Vector3d& x )
        {
            const Eigen::Vector3d r( q_to_r( q ) );
            const double theta = r.norm();
            const double alpha = 1.0 - theta*theta/6.0;
            const double beta = 0.5 - theta*theta/24.0;
            const double gamma = 1.0 / 3.0 - theta*theta/30.0;
            const double delta = -1.0 / 12.0 + theta*theta/180.0;

            return Eigen::Matrix3d(
                -skew_symmetric(x)*(gamma*r*r.transpose()
                - beta*skew_symmetric(r)+alpha*Eigen::Matrix3d::Identity())
                -skew_symmetric(r)*skew_symmetric(x)*(delta*r*r.transpose() 
                + 2.0*beta*Eigen::Matrix3d::Identity()) );
        }
    };

    /** Default std::cout function
    */
    inline std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans)
    {
        /** cout the 6D pose vector (translation and scaled axis orientation) with its associated covariance matrix **/
        base::Vector3d scaled_axis;
        base::AngleAxisd angle_axis (trans.orientation);
        scaled_axis = angle_axis.axis() * angle_axis.angle();
        for (register unsigned short i=0; i<trans.getCovariance().rows(); ++i)
        {
            if (i<3)
            {
                out<<std::fixed<<std::setprecision(5)<<trans.translation[i]<<"\t|";
            }
            else
            {
                out<<std::fixed<<std::setprecision(5)<<scaled_axis[i-3]<<"\t|";
            }
            for (register unsigned short j=0; j<trans.getCovariance().cols(); ++j)
            {
                out<<std::fixed<<std::setprecision(5)<<trans.getCovariance().row(i)[j]<<"\t";
            }
            out<<"\n";
        }
        out.unsetf(std::ios_base::floatfield);
        return out;
    };
} // namespaces

#endif
