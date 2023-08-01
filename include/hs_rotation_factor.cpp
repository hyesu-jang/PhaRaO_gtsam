#include <hs_rotation_factor.h>

namespace gtsam {

/* ************************************************************************* */
PharaoRotFactor::PharaoRotFactor(Key poseKey1, Key poseKey2, double mtheta,
    const SharedNoiseModel& model)
: Base(model, poseKey1, poseKey2), mtheta_(mtheta)
{
}

/* ************************************************************************* */
Vector PharaoRotFactor::evaluateError(const Pose2& pose1, const Pose2& pose2,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
        double hx = 0.0;
        if(pose2.theta()>M_PI/2 && pose1.theta()<-M_PI/2){
            hx = pose2.theta() - 2*M_PI - pose1.theta();
        }
        else if (pose2.theta()<-M_PI/2 && pose1.theta()>M_PI/2){
            hx = pose2.theta() - pose1.theta() + 2*M_PI;
        }
        else{
            hx = pose2.theta() - pose1.theta();
        }
  if (H1) {
    *H1 = Matrix::Zero(1,3);
    (*H1)(0, 2) = -1.0;
  }

  if (H2) {
    *H2 = Matrix::Zero(1,3);
    (*H2)(0, 2) = 1.0;
  }
  return (Vector(1) << hx - mtheta_).finished();
}

/* ************************************************************************* */
bool PharaoRotFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e = dynamic_cast<const This*> (&expected);
  return e != nullptr && Base::equals(*e, tol) && std::abs(this->mtheta_ - e->mtheta_) < tol;
}

/* ************************************************************************* */
void PharaoRotFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "PharaoRotFactor, relative rotation = " << mtheta_ << std::endl;
  Base::print("", keyFormatter);
}
/* ************************************************************************* */

} // \namespace gtsam