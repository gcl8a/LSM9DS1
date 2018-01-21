//
//  LSM9DS1_calibrate.cpp
//  
//
//  Created by Gregory C Lewin on 12/11/17.
//

#include <LSM9DS1.h>
#include <matrix.h>

/*
 * Calibration using method and nomenclature from:
 * http://aerialarithmetic.blogspot.com/2016/03/soft-iron-and-hard-iron-magnetometer.html
 * except that I've reduced the memory footprint by not storing the full matrix D. Instead,
 * I store only the 9 x 9 matrix product DtD = [D' * D] and the 9 x 1 Dt1 = [D' * ones].
 * I do keep the samples for the moment, just to see the transformation.
 */

int LSM9DS1::CalibrateMagnetometer(uint16_t nSamples)
{
    mCond.bias.Zero();
    mCond.condMatrix = float32matrix::Eye(3);

    SerialUSB.println("Sampling");
    
    //we'll use double precision here because we're adding so many values
    dmatrix DtD(9,9);
    dvector Dt1(9);
    
    float32matrix samples(nSamples, 3);
    
    for(int i = 0; i < nSamples; i++)
    {
        while (!IsAvailableMagnetometer()) {} //wait for a new magnetometer reading

        ReadMag();
        float32vector mag = CalcMag();
        
        float x = mag[0];
        float y = mag[1];
        float z = mag[2];

        //for later use
        samples[i][0] = x;
        samples[i][1] = y;
        samples[i][2] = z;
        
        DtD[0][0] += x * x * x * x;
        DtD[0][1] += x * x * y * y;
        DtD[0][2] += x * x * z * z;
        DtD[0][3] += 2.0 * x * x * x * y;
        DtD[0][4] += 2.0 * x * x * x * z;
        DtD[0][5] += 2.0 * x * x * y * z;
        DtD[0][6] += 2.0 * x * x * x;
        DtD[0][7] += 2.0 * x * x * y;
        DtD[0][8] += 2.0 * x * x * z;

        DtD[1][1] += y * y * y * y;
        DtD[1][2] += y * y * z * z;
        DtD[1][3] += 2.0 * y * y * x * y;
        DtD[1][4] += 2.0 * y * y * x * z;
        DtD[1][5] += 2.0 * y * y * y * z;
        DtD[1][6] += 2.0 * y * y * x;
        DtD[1][7] += 2.0 * y * y * y;
        DtD[1][8] += 2.0 * y * y * z;

        DtD[2][2] += z * z * z * z;
        DtD[2][3] += 2.0 * z * z * x * y;
        DtD[2][4] += 2.0 * z * z * x * z;
        DtD[2][5] += 2.0 * z * z * y * z;
        DtD[2][6] += 2.0 * z * z * x;
        DtD[2][7] += 2.0 * z * z * y;
        DtD[2][8] += 2.0 * z * z * z;

        DtD[3][3] += 4.0 * x * y * x * y;
        DtD[3][4] += 4.0 * x * y * x * z;
        DtD[3][5] += 4.0 * x * y * y * z;
        DtD[3][6] += 4.0 * x * y * x;
        DtD[3][7] += 4.0 * x * y * y;
        DtD[3][8] += 4.0 * x * y * z;

        DtD[4][4] += 4.0 * x * z * x * z;
        DtD[4][5] += 4.0 * x * z * y * z;
        DtD[4][6] += 4.0 * x * z * x;
        DtD[4][7] += 4.0 * x * z * y;
        DtD[4][8] += 4.0 * x * z * z;

        DtD[5][5] += 4.0 * y * z * y * z;
        DtD[5][6] += 4.0 * y * z * x;
        DtD[5][7] += 4.0 * y * z * y;
        DtD[5][8] += 4.0 * y * z * z;

        DtD[6][6] += 4.0 * x * x;
        DtD[6][7] += 4.0 * x * y;
        DtD[6][8] += 4.0 * x * z;

        DtD[7][7] += 4.0 * y * y;
        DtD[7][8] += 4.0 * y * z;

        DtD[8][8] += 4.0 * z * z;
        
        Dt1[0] += x * x;
        Dt1[1] += y * y;
        Dt1[2] += z * z;
        Dt1[3] += 2 * x * y;
        Dt1[4] += 2 * x * z;
        Dt1[5] += 2 * y * z;
        Dt1[6] += 2 * x;
        Dt1[7] += 2 * y;
        Dt1[8] += 2 * z;

        SerialUSB.print(i);
        SerialUSB.print('\t');
        SerialUSB.print(x);
        SerialUSB.print('\t');
        SerialUSB.print(y);
        SerialUSB.print('\t');
        SerialUSB.print(z);
        SerialUSB.print('\n');
    }
    
    SerialUSB.println("Done sampling");

    //make DtD symmetric
    for(int i = 1; i < 9; i++)
    {
        for(int j = 0; j < i; j++)
        {
            DtD[i][j] = DtD[j][i];
        }
    }
    
    //this will run out of memory in a hurry...just getting it going first
//    dmatrix D(nSamples, 9);
//    dvector ones(nSamples);
//
//    for(int i = 0; i < nSamples; i++)
//    {
//        D[i][0] = samples[i][0] * samples[i][0];
//        D[i][1] = samples[i][1] * samples[i][1];
//        D[i][2] = samples[i][2] * samples[i][2];
//        D[i][3] = 2 * samples[i][0] * samples[i][1];
//        D[i][4] = 2 * samples[i][2] * samples[i][0];
//        D[i][5] = 2 * samples[i][1] * samples[i][2];
//        D[i][6] = 2 * samples[i][0];
//        D[i][7] = 2 * samples[i][1];
//        D[i][8] = 2 * samples[i][2];
//
//        ones[i] = 1.0;
//    }
//
//    dmatrix DD = D.MakeTranspose() * D;
//
//    for(int i = 0; i < 9; i++)
//    {
//        for(int j = 0; j < 9; j++)
//        {
//            SerialUSB.print(DD[i][j]);
//            SerialUSB.print('\t');
//        }
//        SerialUSB.print('\n');
//    }
    
    SerialUSB.print('\n');

    for(int i = 0; i < 9; i++)
    {
        for(int j = 0; j < 9; j++)
        {
            SerialUSB.print(DtD[i][j]);
            SerialUSB.print('\t');
        }
        SerialUSB.print('\n');
    }
    
    SerialUSB.print('\n');

//    dmatrix Dt = D.MakeTranspose();
//    dvector Pdirect = (Dt * D).FindInverse() * Dt * ones;
    
    dvector P = DtD.Invert() * Dt1;
    
    for(int i = 0; i < 9; i++)
    {
        SerialUSB.print(P[i]);
        SerialUSB.print('\t');
    }
    
    SerialUSB.print('\n');
//
//    for(int i = 0; i < 9; i++)
//    {
//        SerialUSB.print(Pdirect[i]);
//        SerialUSB.print('\t');
//    }
    
    SerialUSB.print('\n');
    SerialUSB.print('\n');

    dmatrix S(3,3);
    dvector b(3);
    
    S[0][0] = P[0];
    S[0][1] = P[3];
    S[0][2] = P[4];
    S[1][0] = P[3];
    S[1][1] = P[1];
    S[1][2] = P[5];
    S[2][0] = P[4];
    S[2][1] = P[5];
    S[2][2] = P[2];

    b[0] = P[6];
    b[1] = P[7];
    b[2] = P[8];
    
    dvector C0 = S.FindInverse() * (b * (-1));
    
    for(int i = 0; i < 3; i++)
    {
        SerialUSB.print(C0[i]);
        SerialUSB.print('\t');
    }
    
    SerialUSB.print('\n');
    
    float w = C0.Dot(S * C0) + C0.Dot(b * 2.0) - 1.0;
    
    dmatrix U = S * (-1.0/w);
    
//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            SerialUSB.print(U[i][j]);
//            SerialUSB.print('\t');
//        }
//        SerialUSB.print('\n');
//    }
    
    dmatrix Q(3,3);
    dmatrix Du(3,3);
    
    U.CalcEigens(Du, Q);

//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            SerialUSB.print(Du[i][j]);
//            SerialUSB.print('\t');
//        }
//        SerialUSB.print('\n');
//    }
    
//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            SerialUSB.print(Q[i][j]);
//            SerialUSB.print('\t');
//        }
//        SerialUSB.print('\n');
//    }
    
    dmatrix sqrtDu = Du;
    for(int i = 0; i < 3; i++)
    {
        sqrtDu[i][i] = sqrt(Du[i][i]);
    }
    
    dmatrix Ainv = Q * sqrtDu * Q.FindInverse();
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            SerialUSB.print(Ainv[i][j]);
            SerialUSB.print('\t');
        }
        SerialUSB.print('\n');
    }
    
    SerialUSB.println("Done computing");
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            mCond.condMatrix[i][j] = Ainv[i][j];
        }
        mCond.bias[i] = C0[i];
    }
    
    for(int i = 0; i < nSamples; i++)
    {
        float32vector sample = samples.GetRow(i);
        float32vector adj = mCond.condMatrix * (sample - mCond.bias);
        
        for(int j = 0; j < 3; j++)
        {
            SerialUSB.print(sample[j]);
            SerialUSB.print('\t');
        }
        
        for(int j = 0; j < 3; j++)
        {
            SerialUSB.print(adj[j]);
            SerialUSB.print('\t');
        }
        SerialUSB.print('\n');
    }
    
    return 0;
}

void LSM9DS1::CalibrateAccelerometerAndGyroBasic(uint16_t samples)
{
    gCond.bias.Zero();
    aCond.bias.Zero();
    
    gCond.condMatrix = float32matrix::Eye(3);
    aCond.condMatrix = float32matrix::Eye(3);
    
    float32vector avgAccel(3);
    float32vector avgGyro(3);
    
    for(uint16_t i = 0; i < samples; i++)
    {
        while(!IsAvailableAccelAndGyro()) {}
        ReadAccel();
        avgAccel += CalcAccel();
        
        ReadGyro();
        avgGyro += CalcGyro();
    }
    
    avgAccel *= (1.0 / samples);
    avgGyro  *= (1.0 / samples);

    avgAccel[2] -= 1.0;
    
    aCond.bias = avgAccel;
    gCond.bias = avgGyro;
    
//    for(int i = 0; i < 3; i++)
//    {
////        for(int j = 0; j < 3; j++)
////        {
////            mCond.condMatrix[i][j] = Ainv[i][j];
////        }
//        aCond.bias[i] = avgAccel[i];
//        gCond.bias[i] = avgGyro[i];
//    }
//
    SerialUSB.print(aCond.bias[0]);
    SerialUSB.print('\t');
    SerialUSB.print(aCond.bias[1]);
    SerialUSB.print('\t');
    SerialUSB.print(aCond.bias[2]);
    SerialUSB.print('\n');

    SerialUSB.print(gCond.bias[0]);
    SerialUSB.print('\t');
    SerialUSB.print(gCond.bias[1]);
    SerialUSB.print('\t');
    SerialUSB.print(gCond.bias[2]);
    SerialUSB.print('\n');
    
    ahrs.Reset();
}

void LSM9DS1::LoadCalibrationDataAccel(const float32vector& data)
{
    aCond.Deserialize(data);
    
    SerialUSB.print(aCond.bias[0]);
    SerialUSB.print('\t');
    SerialUSB.print(aCond.bias[1]);
    SerialUSB.print('\t');
    SerialUSB.print(aCond.bias[2]);
    SerialUSB.print('\n');
    
}

void LSM9DS1::LoadCalibrationDataGyro(const float32vector& data)
{
    gCond.Deserialize(data);

    SerialUSB.print(gCond.bias[0]);
    SerialUSB.print('\t');
    SerialUSB.print(gCond.bias[1]);
    SerialUSB.print('\t');
    SerialUSB.print(gCond.bias[2]);
    SerialUSB.print('\n');
}

void LSM9DS1::LoadCalibrationDataMag(const float32vector& data)
{
    mCond.Deserialize(data);
}

float32vector LSM9DS1::OffloadCalibrationDataAccel(void)
{
    return aCond.Serialize();
}

float32vector LSM9DS1::OffloadCalibrationDataGyro(void)
{
    return gCond.Serialize();
}

float32vector LSM9DS1::OffloadCalibrationDataMag(void)
{
    return mCond.Serialize();
}

