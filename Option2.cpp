#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

using namespace std;
using namespace Eigen;

/**
 * CRTP
 */

class DrakeJoint
{
public:
  virtual Matrix<double, Dynamic, Dynamic> qdotToV(
          const Matrix<double, Dynamic, 1>& q) const = 0;

  virtual Matrix<AutoDiffScalar<VectorXd>, Dynamic, Dynamic> qdotToV(
          const Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1>& q) const = 0;
};


template<typename Derived>
class DrakeJointImpl : public DrakeJoint
{
private:
  Derived& derived;

public:
  DrakeJointImpl(Derived& derived) : derived(derived) { }

  virtual Matrix<double, Dynamic, Dynamic> qdotToV(const Matrix<double, Dynamic, 1>& q) const { return derived.qdotToV(q); }

  virtual Matrix<AutoDiffScalar<VectorXd>, Dynamic, Dynamic> qdotToV(
          const Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1>& q) const { return derived.qdotToV(q); };
};

class FixedAxisOneDoFJoint : public DrakeJointImpl<FixedAxisOneDoFJoint>
{
public:
  FixedAxisOneDoFJoint() : DrakeJointImpl<FixedAxisOneDoFJoint>(*this) { }

  template<typename DerivedQ>
  Matrix<typename DerivedQ::Scalar, Dynamic, Dynamic> qdotToV(const MatrixBase<DerivedQ>& q) const
  {
    Matrix<typename DerivedQ::Scalar, Dynamic, Dynamic> ret(1, 1);
    ret.setIdentity();
    return ret;
  };
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
