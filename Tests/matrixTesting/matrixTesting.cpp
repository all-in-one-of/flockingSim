#include <iostream>
#include "flock_gpu.h"
#include <gtest/gtest.h>
#include <fstream>


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

//TEST(MatrixConstructors,DefaultConstructor)
//{
//    Matrix<int,2,2> mat;
//    Matrix<int,2,2> result{0,0,0,0};
//    EXPECT_TRUE(mat == result);

//}

//TEST(MatrixConstructors,CopyConstructor)
//{
//    Matrix<int,2,2> mat{1,2,3,4};
//    Matrix<int,2,2> mat2;
//    Matrix<int,2,2> result{1,2,3,4};
//    mat2=mat;
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixConstructors,InitializerListConstructor)
//{
//    Matrix<int,2,2> mat{1,2,3,4};

//    Matrix<int,2,2> result;
//    result(1,1)=1;
//    result(1,2)=2;
//    result(2,1)=3;
//    result(2,2)=4;


//    EXPECT_TRUE(mat == result);

//}


//TEST(MatrixOperators,SubScriptOperator)
//{
//    Matrix<int,2,3> mat;
//    mat(1,3)=5;
//    Matrix<int,2,3> result{0,0,5,0,0,0};
//    EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,SubScriptOperatorOutOfRange)
//{
//    Matrix<int,2,3> mat;

//    EXPECT_THROW(mat(1,4)=5, std::out_of_range);

//}

//TEST(MatrixOperators,RowsAccessor)
//{
//    Matrix<int,2,3> mat;

//    EXPECT_TRUE(mat.getRows() == 2);

//}

//TEST(MatrixOperators,ColsAccessor)
//{
//    Matrix<int,2,3> mat;

//    EXPECT_TRUE(mat.getCols() == 3);

//}


//TEST(MatrixOperators,AssignmentOperator)
//{
//    Matrix<int,2,2> mat{1,2,3,4};
//    Matrix<int,2,2> mat2;
//    Matrix<int,2,2> result{1,2,3,4};
//    mat2=mat;
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixOperators,AssignmentOperatorDifferentSizes)
//{
//    Matrix<int,2,3> mat{1,2,3,4,5,6};
//    Matrix<int,2,2> mat2;
//    Matrix<int,2,2> result{1,2,4,5};
//    mat2=mat;
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixOperators,EquilityOperatorTrue)
//{
//    Matrix<int,2,3> mat{1,2,3,4,5,6};
//    Matrix<int,2,3> result{1,2,3,4,5,6};

//    EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,EquilityOperatorFalse)
//{
//    Matrix<int,2,3> mat{1,2,3,4,5,6};
//    Matrix<int,2,3> result{1,2,3,4,5,7};

//    EXPECT_FALSE(mat == result);

//}

//TEST(MatrixOperators,AdditionOperatorMatrix)
//{
//    Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//    Matrix<int,3,3> mat2{2,3,4,5,6,7,8,9,10};
//    Matrix<int,3,3> add;
//    Matrix<int,3,3> result{3,5,7,9,11,13,15,17,19};

//    add=mat+mat2;

//    EXPECT_TRUE(add == result);

//}

//TEST(MatrixOperators,AdditionOperatorScalar)
//{
//     Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//     Matrix<int,3,3> result{5,6,7,8,9,10,11,12,13};

//     mat= mat+ 4;

//     EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,AdditionOperatorNegScalar)
//{
//    Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//    Matrix<int,3,3> result{-3,-2,-1,0,1,2,3,4,5};

//    mat= mat+ -4;

//    EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,SubtractionOperatorMatrix)
//{
//    Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//    Matrix<int,3,3> mat2{2,3,4,5,6,7,8,9,10};
//    Matrix<int,3,3> sub;
//    Matrix<int,3,3> result{-1,-1,-1,-1,-1,-1,-1,-1,-1};

//    sub=mat-mat2;

//    EXPECT_TRUE(sub == result);

//}

//TEST(MatrixOperators,SubtractionOperatorScalar)
//{
//     Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//     Matrix<int,3,3> result{-3,-2,-1,0,1,2,3,4,5};

//     mat= mat- 4;

//     EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,SubtractionOperatorNegScalar)
//{
//    Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};
//    Matrix<int,3,3> result{5,6,7,8,9,10,11,12,13};

//    mat= mat- -4;

//    EXPECT_TRUE(mat == result);

//}

//TEST(MatrixOperators,NegativeOperator)
//{
//    Matrix<int,2,2> mat{1,2,3,4};
//    Matrix<int,2,2> mat2;
//    Matrix<int,2,2> result{-1,-2,-3,-4};
//    mat2=-mat;
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixOperators,MultiplicationOperator)
//{
//    Matrix<int,2,3> mat{1,0,-2,0,3,-1};
//    Matrix<int,3,2> mat2{0,3,-2,-1,0,4};
//    Matrix<int,2,2> mult;
//    Matrix<int,2,2> result{0,-5,-6,-7};
//    mult=mat*mat2;
//    EXPECT_TRUE(mult == result);

//}

//TEST(MatrixOperators,MultiplicationOperatorScalar)
//{
//    Matrix<int,2,2> mat{1,2,3,4};
//    Matrix<int,2,2> mat2;
//    Matrix<int,2,2> result{4,8,12,16};
//    mat2=mat*4;
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixOperators,DivisionOperatorScalar)
//{
//    Matrix<float,2,2> mat{1,2,3,4};
//    Matrix<float,2,2> mat2;
//    Matrix<float,2,2> result{0.25,0.5,0.75,1};
//    mat2=mat/4;
//    EXPECT_TRUE(mat2 == result);

//}



//TEST(MatrixFunctions,MatrixInverse2x2)
//{
//    Matrix<float,2,2> mat{4.0f,7.0f,2.0f,6.0f};
//    Matrix<float,2,2> mat2;
//    Matrix<float,2,2> result{0.6f,-0.7f,-0.2f,0.4f};
//    mat2=mat.inverse();
//    EXPECT_TRUE(mat2 == result);

//}

///// correct results sourced from https://www.mathsisfun.com/algebra/matrix-inverse-row-operations-gauss-jordan.html
//TEST(MatrixFunctions,MatrixInverse3x3)
//{

//    Matrix<float,3,3> mat{3.0f,0.0f,2.0f,2.0f,0.0f,-2.0f,0.0f,1.0f,1.0f};
//    Matrix<float,3,3> mat2;
//    Matrix<float,3,3> result{0.2f,0.2f,0.0f,-0.2f,0.3f,1.0f,0.2f,-0.3f,0.0f};
//    mat2=mat.inverse();
//    EXPECT_TRUE(mat2 == result);

//}

///// correct results sourced from www.mathsisfun.com's matrix calculator
//TEST(MatrixFunctions,MatrixInverse4x4)
//{

//    Matrix<float,4,4> mat{4.0f,0.0f,0.0f,0.0f,0.0f,0.0f,2.0f,0.0f,0.0f,1.0f,2.0f,0.0f,1.0f,0.0f,0.0f,1.0f};
//    Matrix<float,4,4> mat2;
//    Matrix<float,4,4> result{0.25f,0.0f,0.0f,0.0f,0.0f,-1.0f,1.0f,0.0f,0.0f,0.5f,0.0f,0.0f,-0.25f,0.0f,0.0f,1.0f};
//    mat2=mat.inverse();
//    EXPECT_TRUE(mat2 == result);

//}

//TEST(MatrixFunctions,MatrixInverseNonSquare)
//{

//    Matrix<int,2,3> mat{4,3,7,5,6,4};

//    EXPECT_THROW(mat.inverse(), std::out_of_range);

//}

//TEST(MatrixFunctions,MatrixTranspose)
//{
//    Matrix<int,3,2> mat{6,2,4,10,2,7};
//    Matrix<int,2,3> mat2;
//    Matrix<int,2,3> results{6,4,2,2,10,7};

//    mat2=mat.transpose();

//    EXPECT_TRUE(mat2 == results);

//}

//TEST(MatrixFunctions,MatrixOrthogonalTrue)
//{
//    Matrix<int,3,3> mat{1,0,0,0,1,0,0,0,1};

//    EXPECT_TRUE(mat.orthogonal());

//}

//TEST(MatrixFunctions,MatrixOrthogonalFalse)
//{
//    Matrix<int,3,3> mat{4,7,2,3,6,9,6,2,6};

//    EXPECT_FALSE(mat.orthogonal());

//}


//TEST(MatrixFunctions,MatrixDeterminantZero)
//{
//    Matrix<int,3,3> mat{1,2,3,4,5,6,7,8,9};

//    EXPECT_THROW(mat.inverse(),std::out_of_range);

//}

//TEST(MatrixFunctions,MatrixDeterminant2x2)
//{
//    Matrix<int,2,2> mat{4,7,2,6};

//    EXPECT_TRUE(mat.determinant() == 10);

//}


//TEST(MatrixFunctions,MatrixDeterminant3x3)
//{
//    Matrix<int,3,3> mat{6,9,4,5,6,2,5,4,3};

//    EXPECT_TRUE(mat.determinant() == -25);

//}


//TEST(MatrixFunctions,MatrixDeterminant4x4)
//{
//    Matrix<int,4,4> mat{4,0,0,0,0,0,2,0,0,1,2,0,1,0,0,1};

//    EXPECT_TRUE(mat.determinant() == -8);

//}

///// correct results sourced from http://www.purplemath.com
//TEST(MatrixFunctions,MinorMatrix)
//{
//    Matrix<int,4,4> mat{1,2,3,4,2,2,2,4,4,5,6,4,7,8,9,4};
//    Matrix<int,3,3> mat2;
//    Matrix<int,3,3> results{1,2,4,4,5,4,7,8,4};

//    mat2=mat.minorMatrix(2,3);

//    EXPECT_TRUE(mat2 == results);


//}

//TEST(MatrixFunctions,MinorMatrixLowerBoundary)
//{
//    Matrix<int,4,4> mat{1,2,3,4,2,2,2,4,4,5,6,4,7,8,9,4};
//    Matrix<int,3,3> mat2;
//    Matrix<int,3,3> results{2,2,4,5,6,4,8,9,4};

//    mat2=mat.minorMatrix(1,1);

//    EXPECT_TRUE(mat2 == results);

//}

//TEST(MatrixFunctions,MinorMatrixUpperBoundary)
//{
//    Matrix<int,4,4> mat{1,2,3,4,2,2,2,4,4,5,6,4,7,8,9,4};
//    Matrix<int,3,3> mat2;
//    Matrix<int,3,3> results{1,2,3,2,2,2,4,5,6};

//    mat2=mat.minorMatrix(4,4);

//    EXPECT_TRUE(mat2 == results);

//}




















