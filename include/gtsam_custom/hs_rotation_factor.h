#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/OptionalJacobian.h>

#include <optional>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam{
class GTSAM_EXPORT PharaoRotFactor: public NoiseModelFactorN<Pose2, Pose2> {
  double mtheta_; ///< X and Y measurements
  typedef PharaoRotFactor This;
  typedef NoiseModelFactorN<Pose2, Pose2> Base;

public:
  using Base::evaluateError;

  PharaoRotFactor() : mtheta_(0.0) {} /* Default constructor */

  PharaoRotFactor(Key poseKey1, Key poseKey2, double mtheta,
      const SharedNoiseModel& model);

  ~PharaoRotFactor() override {}

  /** h(x)-z */
  Vector evaluateError(const Pose2& pose1, const Pose2& pose2,
      OptionalMatrixType H1, OptionalMatrixType H2) const override;

  /** return the measured */
  inline double mtheta() const { return mtheta_; }

  /** equals specialized to this factor */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override;

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;


private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(mtheta_);
  }
#endif
};

} // \namespace gtsam

