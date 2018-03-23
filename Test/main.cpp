#include <iostream>
#include <gtest/gtest.h>
#include <fstream>
#include <flock_gpu.h>
#include <Flock.h>

#include <vector>
#include <ngl/Vec3.h>

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();

}

TEST(test,gpu)
{
    Flock f(80);
    for(int i = 0; i<f.getNoBoids(); i++ )
    {
        f.getHash();
        f.getCellOcc();

        EXPECT_TRUE(f.m_cellOcc[i] <= f.m_numBoids);
    }


}
