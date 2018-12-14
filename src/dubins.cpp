//Dubins shortest path function
#include <iostream>
#include <math.h>
#include <vector>
#include <assert.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "dubins.hpp"

double sc_th0, sc_thf, sc_Kmax, lambda;
double sc_s1_c, sc_s2_c, sc_s3_c;
double sc_s1, sc_s2, sc_s3;
double s1, s2, s3;
double cline[3];
int pidx;

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Auxiliary utility functions
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// % Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t
// % otherwise
double sinc(double t)
{
    double s;
    if(abs(t)<0.002)
    {
        s = 1-pow(t,2)/6*(1-pow(t,2)/20);
    }
    else
    {
        s = sin(t)/t;
    }
    return s;
}

// % Normalize an angle (in range [0,2*pi))
inline double mod2pi(double angle)
{
    double out = angle;
    while(out<0) 
    {
        out = out + 2*M_PI;
    }
    while(out>=2*M_PI) 
    {
        out = out - 2*M_PI;
    }

    return out;
}

// % Normalize an angular difference (range (-pi, pi])
double rangeSymm(double angle)
{
    double out = angle;
    while(out<= -M_PI)
    {
        out = out+2*M_PI;
    }
    while(out>M_PI)
    {
        out = out-2*M_PI;
    }
    return out;

}

// % Check validity of a solution by evaluating explicitly the 3 equations 
// % defining a Dubins problem (in standard form)
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf)
{
    double x0 = -1;
    double y0 = 0;
    double xf = 1;
    double yf = 0;

    double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;
    double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;
    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

    double Lpos = (s1>0) || (s2>0) || (s3>0);
    bool reply = (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-6) && Lpos;
    return reply;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Functions to plot Dubins curves
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// % Evaluate an arc (circular or straight) composing a Dubins curve, at a 
// % given arc-length s
void circline(double s, double x0, double y0, double th0, double k)
{
    double x = x0 + s*sinc(k*s/2.0)*cos(th0+k*s/2);
    double y = y0 + s*sinc(k*s/2.0)*sin(th0+k*s/2);
    double th = mod2pi(th0+k*s);
    cline[0] = x;
    cline[1] = y;
    cline[2] = th;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Data structures
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// % Create a structure representing an arc of a Dubins curve (straight or
// % circular)
DubinsArc calculateDubinsArc(double x0, double y0, double th0, double k, double L)
{
    DubinsArc c;
    c.x0 = x0;
    c.y0 = y0;
    c.th0 = th0;
    c.k = k;
    c.L = L;
    circline(L,x0,y0,th0,k);
    c.xf = cline[0];
    c.yf = cline[1];
    c.thf = cline[2];
    return c;
}

DubinsCurve calculateDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
{
    DubinsCurve d;
    d.arc1 = calculateDubinsArc(x0,y0,th0,k0,s1);
    d.arc2 = calculateDubinsArc(d.arc1.xf,d.arc1.yf,d.arc1.thf,k1,s2);
    d.arc3 = calculateDubinsArc(d.arc2.xf,d.arc2.yf,d.arc2.thf,k2,s3);
    d.L = d.arc1.L + d.arc2.L + d.arc3.L;
    return d;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Functions to scale and solve Dubins problems
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// % Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
// function [sc_th0, sc_thf, sc_Kmax, lambda] = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax)
void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
{
    //   % find transform parameters
    double dx = xf-x0; 
    double dy = yf-y0;
    double phi = atan2(dy,dx);
    lambda = hypot(dx,dy);

    double C = dx/lambda;
    double S = dy/lambda;
    lambda = lambda/2;

    //   % scale and normalize angles and curvature
    sc_th0 = mod2pi(th0-phi);
    sc_thf = mod2pi(thf-phi);
    sc_Kmax = Kmax*lambda;

    // std::cout<<"Scaled th0: "<<sc_th0<<std::endl;
    // std::cout<<"Scaled thf: "<<sc_thf<<std::endl;
    // std::cout<<"Scaled Kmax: "<<sc_Kmax<<std::endl;
}

// % Scale the solution to the standard problem back to the original problem
void scaleFromStandard(double lambda, double sc_s1_in, double sc_s2_in, double sc_s3_in)
{
    s1 = sc_s1_in*lambda;
    s2 = sc_s2_in*lambda;
    s3 = sc_s3_in*lambda;
}

// % LSL
bool LSL(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    sc_s1_c = invK * mod2pi(temp1 - sc_th0);
    double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * sqrt(temp2);
    sc_s3_c = invK * mod2pi(sc_thf - temp1);
    std::cout << "LSL " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}

// % RSR
bool RSR(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
    sc_s1_c = invK * mod2pi(sc_th0 - temp1);
    double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * sqrt(temp2);
    sc_s3_c = invK * mod2pi(temp1 - sc_thf);
    std::cout << "RSR " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}

// % LSR
bool LSR(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(-C, S);
    double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * sqrt(temp3);
    double temp2 = -atan2(-2, sc_s2_c * sc_Kmax);
    sc_s1_c = invK * mod2pi(temp1 + temp2 - sc_th0);
    sc_s3_c = invK * mod2pi(temp1 + temp2 - sc_thf);
    std::cout << "LSR " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}


// % RSL
bool RSL(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * sqrt(temp3);
    double temp2 = atan2(2, sc_s2_c * sc_Kmax);
    sc_s1_c = invK * mod2pi(sc_th0 - temp1 + temp2);
    sc_s3_c = invK * mod2pi(sc_thf - temp1 + temp2);
    std::cout << "RSL " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}

// % RLR
bool RLR(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * mod2pi(2 * M_PI - acos(temp2));
    sc_s1_c = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2_c * sc_Kmax);
    sc_s3_c = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2_c - sc_s1_c));
    std::cout << "RLR " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}

// % LRL
bool LRL(double sc_th0, double sc_thf, double sc_Kmax)
{
    double invK = 1 / sc_Kmax;
    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1)
    {
        sc_s1_c = 0; 
        sc_s2_c = 0; 
        sc_s3_c = 0;
        return false;
    }
    sc_s2_c = invK * mod2pi(2 * M_PI - acos(temp2));
    sc_s1_c = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2_c * sc_Kmax);
    sc_s3_c = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2_c - sc_s1_c));
    std::cout << "LRL " << sc_s1_c << ", " << sc_s2_c << ", " << sc_s3_c << std::endl;
    return true;

}

// % Solve the Dubins problem for the given input parameters.
// % Return the type and the parameters of the optimal curve
// function [pidx, curve] = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax)
DubinsCurve dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax)
{
    //   % Compute params of standard scaled problem
    //   [sc_th0, sc_thf, sc_Kmax, lambda] = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);

    scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);

    //   % Define the functions corresponding to the different primitives, and the
    //   % corresponding curvatue signs 
    std::vector<std::vector<int>> ksigns = {
                                            { 1,  0,  1}, //LSL
                                            {-1,  0, -1}, //RSR
                                            { 1,  0, -1}, //LSR
                                            {-1,  0,  1}, //RSL
                                            {-1,  1, -1}, //RLR
                                            { 1, -1,  1}  //LRL                                    
    };

    //   % Try all the possible primitives, to find the optimal solution
    pidx = -1;
    double Lcur, L = 100.0; //check what value to put here
    int i = 1;
    bool ok = false;
    while(i<7)
    {
        if(i==1) ok = LSL(sc_th0,sc_thf,sc_Kmax);
        else if(i==2) ok = RSR(sc_th0,sc_thf,sc_Kmax);
        else if(i==3) ok = LSR(sc_th0,sc_thf,sc_Kmax);
        else if(i==4) ok = RSL(sc_th0,sc_thf,sc_Kmax);
        else if(i==5) ok = RLR(sc_th0,sc_thf,sc_Kmax);
        else if(i==6) ok = LRL(sc_th0,sc_thf,sc_Kmax);
        else continue;
        Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
        if(ok && Lcur<L)
        {
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;
        }
        ++i;
    }
    

    // std::cout << "New curve: " << L << std::endl;
    // std::cout << "Curve sc_s1: " << sc_s1 << std::endl;
    // std::cout << "Curve sc_s2: " << sc_s2 << std::endl;
    // std::cout << "Curve sc_s3: " << sc_s3 << std::endl;


    DubinsCurve curve;

    if(pidx > 0)
    {
    //     % Transform the solution to the problem in standard form to the 
    //     % solution of the original problem (scale the lengths)  
        scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);
        
        // std::cout << "Curve s1: " << s1 << std::endl;
        // std::cout << "Curve s2: " << s2 << std::endl;
        // std::cout << "Curve s3: " << s3 << std::endl;

    //     % Construct the Dubins curve object with the computed optimal parameters
        curve = calculateDubinsCurve(x0, y0, th0, s1, s2, s3, (ksigns[pidx-1][0])*Kmax, (ksigns[pidx-1][1])*Kmax, (ksigns[pidx-1][2])*Kmax);
        
    //     % Check the correctness of the algorithm
        // assert(check(sc_s1, ksigns[pidx][0]*sc_Kmax, sc_s2, ksigns[pidx][1]*sc_Kmax, sc_s3, ksigns[pidx][2]*sc_Kmax, sc_th0, sc_thf));
    }   

    return curve;

}