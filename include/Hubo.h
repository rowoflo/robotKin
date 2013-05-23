/*
 -------------------------------------------------------------------------------
 Hubo.h
 robotTest Project
 
 CLASS NAME:
    Hubo
 
 DESCRIPTION:
    description...
 
 FILES:
    Hubo.h
    Hubo.cpp

 DEPENDENCIES:
    
 
 CONSTRUCTORS:
    Hubo();    
 
 PROPERTIES:
    prop1 - description... .

    prop2 - description... .
 
 METHODS:
    type method1(type arg1);
        Description... .
            arg1 - description... .
 
 NOTES:
 
 
 EXAMPLES:
    Example 1: description
    ----------------------------------------------------------------------------
    code...
    ----------------------------------------------------------------------------
 
 
 VERSIONS:
    1.0 - 5/20/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Hubo_h_
#define _Hubo_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Robot.h"
#include "Linkage.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace golems;


//------------------------------------------------------------------------------
// Typedefs
//------------------------------------------------------------------------------
typedef Matrix< double, 4, 1 > Vector4d;
typedef Matrix< double, 5, 1 > Vector5d;
typedef Matrix< double, 6, 1 > Vector6d;
typedef Matrix< double, 25, 1 > Vector25d;
typedef Matrix<double, 1, 2> Matrix12d;
typedef Matrix<double, 6, 2>  Matrix62d;
typedef Matrix< double, 6, 6 > Matrix66d;

enum {
    SIDE_LEFT,
    SIDE_RIGHT
};


class Hubo : public Robot
{
public:
    //--------------------------------------------------------------------------
    // Hubo Lifecycle
    //--------------------------------------------------------------------------
    // Constructors
    Hubo();
    
    // Destructor
    virtual ~Hubo();
    
    
    //--------------------------------------------------------------------------
    // Hubo Public Member Functions
    //--------------------------------------------------------------------------
    bool leftArmAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
    bool rightArmAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
    
    void armFK(Isometry3d& B, const Vector6d& q, size_t side);
    bool armAnalyticalIK(VectorXd& q, const Isometry3d& B, const Vector6d& qPrev, size_t side);
    
    
    bool leftLegAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
    bool rightLegAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
    
    void legFK(Isometry3d& B, const Vector6d& q, size_t side);
    bool legAnalyticalIK(VectorXd& q, const Isometry3d& B, const Vector6d& qPrev, size_t side);
    
    //--------------------------------------------------------------------------
    // Hubo Public Member Variables
    //--------------------------------------------------------------------------
    // Lengths
    double torsoLength;
    VectorXd armLengths;
    VectorXd legLengths;
    
    // Limits
    MatrixX2d torsoLimits;
    MatrixX2d leftArmLimits;
    MatrixX2d rightArmLimits;
    MatrixX2d leftLegLimits;
    MatrixX2d rightLegLimits;
    
    // Offsets
    double torsoOffset;
    VectorXd leftArmOffsets;
    VectorXd rightArmOffsets;
    VectorXd leftLegOffsets;
    VectorXd rightLegOffsets;
    
    //------------------------------------------------------------------------------
    // Hubo Public Helper Functions
    //------------------------------------------------------------------------------
    static inline double mod(double x, double y) {
        if (0 == y)
            return x;
        return x - y * floor(x/y);
    }
    
    static inline double wrapToPi(double fAng) {
        return mod(fAng + M_PI, 2*M_PI) - M_PI;
    }
    
private:
    //--------------------------------------------------------------------------
    // Hubo Private Member Functions
    //--------------------------------------------------------------------------
    void initialize();
    Linkage initializeTorso();
    Linkage initializeLeftArm();
    Linkage initializeRightArm();
    Linkage initializeLeftLeg();
    Linkage initializeRightLeg();
    
    //--------------------------------------------------------------------------
    // Hubo Private Member Variables
    //--------------------------------------------------------------------------
    double zeroSize;
    
    
}; // class Hubo



#endif


