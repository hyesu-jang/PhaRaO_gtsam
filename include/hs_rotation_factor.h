#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam{
class GTSAM_EXPORT PharaoRotFactor: public NoiseModelFactor2<Pose2, Pose2> {
  double mtheta_; ///< X and Y measurements
  typedef PharaoRotFactor This;
  typedef NoiseModelFactor2<Pose2, Pose2> Base;

public:

  PharaoRotFactor() : mtheta_(0.0) {} /* Default constructor */

  PharaoRotFactor(Key poseKey1, Key poseKey2, double mtheta,
      const SharedNoiseModel& model);

  ~PharaoRotFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** h(x)-z */
  Vector evaluateError(const Pose2& pose1, const Pose2& pose2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override;

  /** return the measured */
  inline double mtheta() const { return mtheta_; }

  /** equals specialized to this factor */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override;

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(mtheta_);
  }
};

} // \namespace gtsam

