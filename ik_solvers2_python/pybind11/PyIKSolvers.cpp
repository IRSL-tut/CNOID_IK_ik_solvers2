/**
   @author YoheiKakiuchi
*/

#include <cnoid/PyUtil>
//
//#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
//
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

#include <ik_constraint2/IKConstraint.h>
//#include <ik_constraint2/ORConstraint.h>
//#include <ik_constraint2/ANDConstraint.h>
#include <ik_constraint2/PositionConstraint.h>
//#include <ik_constraint2/RegionConstraint.h>
#include <ik_constraint2/COMConstraint.h>
#include <ik_constraint2/AngularMomentumConstraint.h>
#include <ik_constraint2/JointAngleConstraint.h>
//#include <ik_constraint2/JointRegionConstraint.h>
#include <ik_constraint2/JointLimitConstraint.h>
//#include <ik_constraint2/JointDisplacementConstraint.h>
#include <ik_constraint2/JointVelocityConstraint.h>
//#include <ik_constraint2/CollisionConstraint.h>
#include <ik_constraint2/ClientCollisionConstraint.h>
//#include <ik_constraint2/KeepCollisionConstraint.h>

#include <memory>
#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;
namespace ikc = ik_constraint2;
//IKConstraint
//AngularMomentum
//COM
//COMVelocity
//ClientCollision
//JointAngle
//JointLimit
//JointVelocity
//Position

typedef std::shared_ptr<ikc::IKConstraint >              IKConstraintPtr;
typedef std::shared_ptr<ikc::AngularMomentumConstraint > AngularMomentumConstraintPtr;
typedef std::shared_ptr<ikc::COMConstraint >             COMConstraintPtr;
//typedef std::shared_ptr<IK::COMVelocityConstraint >     COMVelocityConstraintPtr;
typedef std::shared_ptr<ikc::ClientCollisionConstraint > ClientCollisionConstraintPtr;
typedef std::shared_ptr<ikc::JointAngleConstraint >      JointAngleConstraintPtr;
typedef std::shared_ptr<ikc::JointLimitConstraint >      JointLimitConstraintPtr;
typedef std::shared_ptr<ikc::JointVelocityConstraint >   JointVelocityConstraintPtr;
typedef std::shared_ptr<ikc::PositionConstraint >        PositionConstraintPtr;

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

class Constraints
{
public:
  std::vector< IKConstraintPtr > ikc_list;

public:
  Constraints() { };

public:
  void push_back(IKConstraintPtr &ptr) {
    ikc_list.push_back(ptr);
  }
  size_t size() { return ikc_list.size(); }
  IKConstraintPtr &at(int i) { return ikc_list.at(i); }
};

typedef std::shared_ptr < Constraints > ConstraintsPtr;
typedef std::vector < std::shared_ptr < Constraints > > ConstraintsPtrList;

////
typedef std::shared_ptr< prioritized_qp_base::Task > TaskPtr;
class Tasks
{
public:
  std::vector< TaskPtr > tasks;

public:
  Tasks() {};

public:
  void push_back(TaskPtr &ptr) {
    tasks.push_back(ptr);
  }
  size_t size() { return tasks.size(); }
  TaskPtr &at(int i) { return tasks.at(i); }
};
typedef std::shared_ptr < Tasks > TasksPtr;

//std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)>
void _local_taskGeneratorFunc (std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel)
{
  std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
  if(!taskOSQP){
    task = std::make_shared<prioritized_qp_osqp::Task>();
    taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
  }
  taskOSQP->settings().verbose = debugLevel;
  taskOSQP->settings().max_iter = 4000;
  taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
  taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
  taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
}

int prioritized_solveIKLoop(const std::vector<cnoid::LinkPtr>& variables,
         const ConstraintsPtrList &lst,
         TasksPtr& prevTasks,
         size_t max_iteration, double wn, int debugLevel, double dt)
         //size_t max_iteration = 1,
         //double wn = 1e-6,
         //int debugLevel = 0,
         //double dt = 0.1
{
  std::vector<std::vector < IKConstraintPtr> > ikc_list_;
  for(int i = 0; i < lst.size(); i++) {
    std::vector < IKConstraintPtr> &vec = lst[i]->ikc_list;
    std::vector < IKConstraintPtr> copied_vec;
    for(int j = 0; j < vec.size(); j++) {
      copied_vec.push_back(vec[j]);
    }
    ikc_list_.push_back(copied_vec);
  }
  std::vector< TaskPtr > prevTasks_;
  prioritized_inverse_kinematics_solver2::IKParam param;
  param.maxIteration = max_iteration;
  param.wn = wn;
  //param.we = we;
  param.debugLevel = debugLevel;
  param.dt = dt;
  int ret = prioritized_inverse_kinematics_solver2::solveIKLoop
    (variables, ikc_list_, prevTasks_, param,
     nullptr,
     static_cast < std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> > ( &_local_taskGeneratorFunc)
    );

  prevTasks->tasks.clear();
  for(int i = 0; i < prevTasks_.size(); i++) {
    prevTasks->tasks.push_back(prevTasks_[i]);
  }

  return ret;
}

class pyIKConstraint : public ikc::IKConstraint
{
public:
  using ikc::IKConstraint::IKConstraint;

  // trampoline (one for each virtual function)
#if 0
  virtual bool isSatisfied () override {
    PYBIND11_OVERLOAD_PURE(
      bool, /* Return type */
      ikc::IKConstraint,      /* Parent class */
      isSatisfied       /* Name of function in C++ (must match Python name) */
    );
  }
#endif
  virtual void updateBounds () override {
    PYBIND11_OVERLOAD_PURE(
      void, /* Return type */
      ikc::IKConstraint,      /* Parent class */
      updateBounds       /* Name of function in C++ (must match Python name) */
    );
  }
  virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override {
    PYBIND11_OVERLOAD_PURE(
      void, /* Return type */
      ikc::IKConstraint,      /* Parent class */
      updateJacobian        /* Name of function in C++ (must match Python name) */
    );
  }
  virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
     PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<IKConstraint>, /* Return type */
      ikc::IKConstraint,      /* Parent class */
      clone        /* Name of function in C++ (must match Python name) */
    );
  }
};


PYBIND11_MODULE(IKSolvers, m)
{
    m.doc() = "fullbody inverse kinematics module";

    py::module::import("cnoid.Util");

    py::class_< ikc::IKConstraint, IKConstraintPtr, pyIKConstraint > (m, "IKConstraint")
      .def(py::init<>())
      .def_property("debugLevel",
                    (int & (ikc::IKConstraint::*)())&ikc::IKConstraint::debugLevel,
                    [] (ikc::IKConstraint &self, int &in) { self.debugLevel() = in; })
      .def("isSatisfied", &ikc::IKConstraint::isSatisfied)
      ;

    py::class_< Constraints, ConstraintsPtr > (m, "Constraints")
      .def(py::init<>())
      .def("size", &Constraints::size)
      .def("at", &Constraints::at)
      .def("__iter__", [](const Constraints &s) { return py::make_iterator(s.ikc_list.begin(), s.ikc_list.end()); },
           py::keep_alive<0, 1>())
      .def("push_back", (void (Constraints::*)(IKConstraintPtr &)) &Constraints::push_back)
      ;

    //py::class_< prioritized_qp_base::Task, TaskPtr > (m, "Task")
    //  .def(py::init<>())
    //  ;
    py::class_< Tasks, TasksPtr > (m, "Tasks")
      .def(py::init<>())
      //.def("size", &Tasks::size)
      //.def("at", &Tasks::at)
      //.def("__iter__", [](const Tasks &s) { return py::make_iterator(s.tasks.begin(), s.tasks.end()); },
      //     py::keep_alive<0, 1>())
      //.def("push_back", (void (Tasks::*)(TaskPtr &)) &Tasks::push_back)
      ;

    py::class_<ikc::AngularMomentumConstraint, AngularMomentumConstraintPtr, ikc::IKConstraint > (m, "AngularMomentumConstraint")
      .def(py::init<>());

    py::class_<ikc::COMConstraint, COMConstraintPtr, ikc::IKConstraint > (m, "COMConstraint")
      .def(py::init<>())
      .def_property("A_robot",
                    (cnoid::BodyPtr & (ikc::COMConstraint::*)()) &ikc::COMConstraint::A_robot,
                    //&ikc::COMConstraint::set_A_robot)
                    [](ikc::COMConstraint &self, cnoid::BodyPtr &in) { self.A_robot() = in; })
      .def_property("B_robot",
                    (cnoid::BodyPtr & (ikc::COMConstraint::*)()) &ikc::COMConstraint::B_robot,
                    //&ikc::COMConstraint::set_B_robot)
                    [](ikc::COMConstraint &self, cnoid::BodyPtr &in) { self.B_robot() = in; })
      .def_property("A_localp",
                    (cnoid::Vector3 & (ikc::COMConstraint::*)()) &ikc::COMConstraint::A_localp,
                    //&ikc::COMConstraint::set_A_localp)
                    [](ikc::COMConstraint &self, cnoid::Vector3 &in) { self.A_localp() = in; })
      .def_property("B_localp",
                    (cnoid::Vector3 & (ikc::COMConstraint::*)()) &ikc::COMConstraint::B_localp,
                    //&ikc::COMConstraint::set_B_localp)
                    [](ikc::COMConstraint &self, cnoid::Vector3 &in) { self.B_localp() = in; })
      .def_property("eval_R",
                    (cnoid::Matrix3d & (ikc::COMConstraint::*)()) &ikc::COMConstraint::eval_R,
                    //&ikc::COMConstraint::set_eval_R)
                    [](ikc::COMConstraint &self, cnoid::Matrix3d &in) { self.eval_R() = in; })
      .def_property("maxError",
                    (cnoid::Vector3 & (ikc::COMConstraint::*)()) &ikc::COMConstraint::maxError,
                    //&ikc::COMConstraint::set_maxError)
                    [](ikc::COMConstraint &self, cnoid::Vector3 &in) { self.maxError() = in; })
      .def_property("precision",
                    (double & (ikc::COMConstraint::*)()) &ikc::COMConstraint::precision,
                    //&ikc::COMConstraint::set_precision)
                    [](ikc::COMConstraint &self, double in) { self.precision() = in; })
      .def_property("weight",
                    (cnoid::Vector3 & (ikc::COMConstraint::*)()) &ikc::COMConstraint::weight,
                    //&ikc::COMConstraint::set_weight)
                    [](ikc::COMConstraint &self, cnoid::Vector3 &in) { self.weight() = in; })
      ;
#if 0
    py::class_<ikc::COMVelocityConstraint, COMVelocityConstraintPtr, ikc::IKConstraint > (m, "COMVelocityConstraint")
      .def(py::init<>());
#endif
    py::class_<ikc::ClientCollisionConstraint, ClientCollisionConstraintPtr, ikc::IKConstraint > (m, "ClientCollisionConstraint")
      .def(py::init<>());

    //py::class_<ikc::CollisionConstraint, CollisionConstraintPtr > (m, "CollisionConstraint")
    //  .def(py::init<>());
    py::class_<ikc::JointAngleConstraint, JointAngleConstraintPtr, ikc::IKConstraint > (m, "JointAngleConstraint")
      .def(py::init<>())
      .def_property("joint",
                    (cnoid::LinkPtr & (ikc::JointAngleConstraint::*)()) &ikc::JointAngleConstraint::joint,
                    //&ikc::JointAngleConstraint::set_joint)
                    [](ikc::JointAngleConstraint &self, cnoid::LinkPtr &in) { self.joint() = in; })
      .def_property("targetq",
                    (double & (ikc::JointAngleConstraint::*)()) &ikc::JointAngleConstraint::targetq,
                    //&ikc::JointAngleConstraint::set_targetq)
                    [](ikc::JointAngleConstraint &self, double &in) { self.targetq() = in; })
      .def_property("maxError",
                    (double & (ikc::JointAngleConstraint::*)()) &ikc::JointAngleConstraint::maxError,
                    //&ikc::JointAngleConstraint::set_maxError)
                    [](ikc::JointAngleConstraint &self, double &in) { self.maxError() = in; })
      .def_property("precision",
                    (double & (ikc::JointAngleConstraint::*)()) &ikc::JointAngleConstraint::precision,
                    //&ikc::JointAngleConstraint::set_precision)
                    [](ikc::JointAngleConstraint &self, double &in) { self.precision() = in; })
      .def_property("weight",
                    (double & (ikc::JointAngleConstraint::*)()) &ikc::JointAngleConstraint::weight,
                    //&ikc::JointAngleConstraint::set_weight)
                    [](ikc::JointAngleConstraint &self, double &in) { self.weight() = in; })
      ;

    py::class_<ikc::JointLimitConstraint, JointLimitConstraintPtr, ikc::IKConstraint > (m, "JointLimitConstraint")
      .def(py::init<>())
      .def_property("joint",
                    (cnoid::LinkPtr & (ikc::JointLimitConstraint::*)()) &ikc::JointLimitConstraint::joint,
                    //&ikc::JointLimitConstraint::set_joint)
                    [](ikc::JointLimitConstraint &self, cnoid::LinkPtr &in) { self.joint() = in; })
      .def_property("maxError",
                    (double & (ikc::JointLimitConstraint::*)()) &ikc::JointLimitConstraint::maxError,
                    //&ikc::JointLimitConstraint::set_maxError)
                    [](ikc::JointLimitConstraint &self, double &in) { self.maxError() = in; })
      .def_property("precision",
                    (double & (ikc::JointLimitConstraint::*)()) &ikc::JointLimitConstraint::precision,
                    //&ikc::JointLimitConstraint::set_precision)
                    [](ikc::JointLimitConstraint &self, double &in) { self.precision() = in; })
      .def_property("weight",
                    (double & (ikc::JointLimitConstraint::*)()) &ikc::JointLimitConstraint::weight,
                    //&ikc::JointLimitConstraint::set_weight)
                    [](ikc::JointLimitConstraint &self, double &in) { self.weight() = in; })
      ;

    py::class_<ikc::JointVelocityConstraint, JointVelocityConstraintPtr, ikc::IKConstraint > (m, "JointVelocityConstraint")
      .def(py::init<>());

    py::class_<ikc::PositionConstraint, PositionConstraintPtr, ikc::IKConstraint > (m, "PositionConstraint")
      .def(py::init<>())
      .def_property("A_link",
                    (cnoid::LinkPtr & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::A_link,
                    //&ikc::PositionConstraint::set_A_link)
                    [](ikc::PositionConstraint &self, cnoid::LinkPtr &in) { self.A_link() = in; })
      .def_property("B_link",
                    (cnoid::LinkPtr & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::B_link,
                    //&ikc::PositionConstraint::set_B_link)
                    [](ikc::PositionConstraint &self, cnoid::LinkPtr &in) { self.B_link() = in; })
      .def_property("eval_link",
                    (cnoid::LinkPtr & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::eval_link,
                    //&ikc::PositionConstraint::set_eval_link)
                    [](ikc::PositionConstraint &self, cnoid::LinkPtr &in) { self.eval_link() = in; })
      .def_property("A_localpos",
                    //(cnoid::Position & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::A_localpos,
                    //&ikc::PositionConstraint::set_A_localpos)
                    [](ikc::PositionConstraint& self) -> Isometry3::MatrixType& { return self.A_localpos().matrix(); },
                    [](ikc::PositionConstraint& self, Eigen::Ref<const Matrix4RM> in_pos) {
                      Position p(in_pos); self.A_localpos() = p; })
      .def_property("B_localpos",
                    //(cnoid::Position & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::B_localpos,
                    //&ikc::PositionConstraint::set_B_localpos)
                    [](ikc::PositionConstraint& self) -> Isometry3::MatrixType& { return self.B_localpos().matrix(); },
                    [](ikc::PositionConstraint& self, Eigen::Ref<const Matrix4RM> in_pos) {
                      Position p(in_pos); self.B_localpos() = p; })
      .def_property("maxError",
                    (cnoid::Vector6 & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::maxError,
                    //&ikc::PositionConstraint::set_maxError)
                    [](ikc::PositionConstraint &self, cnoid::Vector6 &in) { self.maxError() = in; })
      .def_property("precision",
                    (double & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::precision,
                    //&ikc::PositionConstraint::set_precision)
                    [](ikc::PositionConstraint &self, double &in) { self.precision() = in; })
      .def_property("weight",
                    (cnoid::Vector6 & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::weight,
                    //&ikc::PositionConstraint::set_weight)
                    [](ikc::PositionConstraint &self, cnoid::Vector6 &in) { self.weight() = in; })
      .def_property("eval_localR",
                    (cnoid::Matrix3d & (ikc::PositionConstraint::*)()) &ikc::PositionConstraint::eval_localR,
                    //&ikc::PositionConstraint::set_eval_localR)
                    [](ikc::PositionConstraint &self, cnoid::Matrix3d &in) { self.eval_localR() = in; })
      ;

    m.def("prioritized_solveIKLoop", &prioritized_solveIKLoop,
          py::arg("variables"),
          py::arg("constraints_list"),
          py::arg("prev_tasks"),
          py::arg("max_iteration") = 1,
          py::arg("wn") = 1e-6,
          py::arg("debug_level") = 0,
          py::arg("dt") = 0.1);
}
