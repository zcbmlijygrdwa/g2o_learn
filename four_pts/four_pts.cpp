#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"

#include "g2o/types/slam2d/vertex_point_xy.h"

#include <iostream>

using namespace std;
using namespace g2o;

class MyVertexXY : public BaseVertex <2, Eigen::Vector2d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            //MyVertexXY()
            //{

            //}

            MyVertexXY():BaseVertex <2, Eigen::Vector2d>()
        {
        }

        bool read(std::istream& is)
        {
            is >> _estimate[0] >> _estimate[1];
            return true;

        }

        bool write(std::ostream& os) const
        {
            os << estimate()(0) << " " << estimate()(1);
            return os.good();
        }

        void setToOriginImpl()
        {
            _estimate.setZero();
        }

        void oplusImpl(const number_t* update)
        {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
        }

};

class MyBinaryEdge : public BaseBinaryEdge <1, double, MyVertexXY, MyVertexXY>
{
    public:
        bool read(std::istream& is)
        {
            double p;
            is >> p;
            setMeasurement(p);
            is >> information()(0,0);
            return true;

        }

        bool write(std::ostream& os) const
        {
            double p = measurement();
            os << p;
            os << " " << information()(0, 0);
            return os.good();
        }
        void computeError()
        {
            const MyVertexXY* v1 = static_cast<const MyVertexXY*>(_vertices[0]);
            Eigen::Vector2d myEstimate1 = v1->estimate();

            const MyVertexXY* v2 = static_cast<const MyVertexXY*>(_vertices[1]);
            Eigen::Vector2d myEstimate2 = v2->estimate();

            //cout<<"myEstimate1 = "<<myEstimate1<<endl;
            //cout<<"myEstimate2 = "<<myEstimate2<<endl;
            //cout<<"_measurement = "<<_measurement<<endl;




            _error = Eigen::Matrix<double,1,1>();

            _error(0,0) = _measurement - sqrt(((myEstimate1[0]-myEstimate2[0])*(myEstimate1[0]-myEstimate2[0]) + (myEstimate1[1]-myEstimate2[1])*(myEstimate1[1]-myEstimate2[1])));

            //_error(0,0) = 7;

            cout<<"error = "<<_error<<endl;

            //_error = Eigen::VectorXd(_error_d);

            //_error = 0;

        }


};

#define MAXITERATION 50
int main()
{

    cout<<"Helllo g2o four points example"<<endl;



    // create the linear solver
    std::unique_ptr<BlockSolverX::LinearSolverType> linearSolver(new LinearSolverCSparse<BlockSolverX::PoseMatrixType>());

    // create the block solver on the top of the linear solver
    std::unique_ptr<BlockSolverX> blockSolver(new BlockSolverX(std::move(linearSolver)));

    //create the algorithm to carry out the optimization
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(std::move(blockSolver));


    // create the optimizer
    // allocating the optimizer
    SparseOptimizer optimizer;

    optimizer.setAlgorithm(optimizationAlgorithm);

    // adding the odometry to the optimizer
    // first adding all the vertices
    cerr << "Optimization: Adding robot poses ... "<<endl;

    MyVertexXY* v1 = new MyVertexXY();
    v1->setId(1);
    v1->setEstimate(Eigen::Vector2d(0.01,0.012));
    optimizer.addVertex(v1);


    MyVertexXY* v2 = new MyVertexXY();
    v2->setId(2);
    v2->setEstimate(Eigen::Vector2d(0,1.9));
    optimizer.addVertex(v2);

    MyBinaryEdge* e = new MyBinaryEdge(); 

    e -> setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v1));
    e -> setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v2));

    cout<<"optimizer.vertices().find(0) = "<<optimizer.vertices().size()<<endl;
    e -> setMeasurement(2);
    e -> information() = Eigen::Matrix<double,1,1>::Identity();
    optimizer.addEdge(e);
    optimizer.initializeOptimization();
    cout << "optimizer.initializeOptimization" << endl;

    optimizer.optimize(10); 
    
    cout<<"v1->estimate() = "<<v1->estimate()<<endl;
    cout<<"v2->estimate() = "<<v2->estimate()<<endl;
    return 0;
}
