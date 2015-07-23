#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

using namespace std;
using namespace Eigen;

/**
 * based on http://stackoverflow.com/a/5872633/2228557
 */

class QdotToV
{
public:
  virtual Matrix<double, Dynamic, Dynamic> operator()(
          const Matrix<double, Dynamic, 1>& q) const = 0;

  virtual Matrix<AutoDiffScalar<VectorXd>, Dynamic, Dynamic> operator()(
          const Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1>& q) const = 0;

  virtual ~QdotToV() { }
};

template<typename DrakeJointType>
struct QdotToVImplementation : QdotToV
{
  const DrakeJointType& joint;

  QdotToVImplementation(const DrakeJointType& joint) : joint(joint) { };

  virtual Matrix<double, Dynamic, Dynamic> operator()(
          const Matrix<double, Dynamic, 1>& q) const
  {
    return joint.qdotToVImplementation(q);
  }

  virtual Matrix<AutoDiffScalar<VectorXd>, Dynamic, Dynamic> operator()(
          const Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1>& q) const
  {
    return joint.qdotToVImplementation(q);
  }
  
  virtual ~QdotToVImplementation() { }
};

class DrakeJoint
{
public:
  QdotToV& qdotToV;

  DrakeJoint(QdotToV& qdotToV) : qdotToV(qdotToV) { };

  virtual ~DrakeJoint() {delete &qdotToV; }
};

class FixedAxisOneDoFJoint : public DrakeJoint
{
private:
  template <typename Derived>
  Matrix<typename Derived::Scalar, Dynamic, Dynamic> qdotToVImplementation(const MatrixBase<Derived>& q) const {
    Matrix<typename Derived::Scalar, Dynamic, Dynamic> ret(1, 1);
    ret.setIdentity();
    return ret;
  };

public:
  FixedAxisOneDoFJoint() : DrakeJoint(*(new QdotToVImplementation<FixedAxisOneDoFJoint>(*this))) { }
  friend struct QdotToVImplementation<FixedAxisOneDoFJoint>;
};

template <typename DerivedQ>
Matrix<typename DerivedQ::Scalar, Dynamic, Dynamic> doStuff(const DrakeJoint& joint, const MatrixBase<DerivedQ>& q) {
  return joint.qdotToV(q.derived());
}

int main()
{
  FixedAxisOneDoFJoint fjoint;
  const DrakeJoint& joint = fjoint;

  VectorXd q_double(1);
  q_double[0] = 2.0;
  cout << doStuff(joint, q_double) << endl;

  Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1> q_autodiff(1);
  q_autodiff[0] = 2.0;
  cout << doStuff(joint, q_autodiff).value() << endl;

  return 0;
}
