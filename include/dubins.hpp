#ifndef LAR_DUBINS_HPP
#define LAR_DUBINS_HPP

//Forward declared dependencies
extern double sc_th0, sc_thf, sc_Kmax, lambda;
extern double sc_s1_c, sc_s2_c, sc_s3_c;
extern double sc_s1, sc_s2, sc_s3;
extern double s1, s2, s3;

extern double cline[3];
extern int pidx;

struct DubinsArc {
  double x0;
  double y0;
  double th0;
  double k;
  double L;
  double xf;
  double yf;
  double thf;
} ;

struct DubinsCurve {
    DubinsArc arc1;
    DubinsArc arc2;
    DubinsArc arc3;
    double L;
} ;

//Included dependencies
//

//Classes
//

//Function declarations
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Auxiliary utility functions
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double sinc(double t);
inline double mod2pi(double angle);
double rangeSymm(double angle);
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Data structures
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DubinsArc calculateDubinsArc(double x0, double y0, double th0, double k, double L);
DubinsCurve calculateDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);
void circline(double s, double x0, double y0, double th0, double k);

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// % Functions to scale and solve Dubins problems
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);
void scaleFromStandard(double lambda, double sc_s1_in, double sc_s2_in, double sc_s3_in);
bool LSL(double sc_th0, double sc_thf, double sc_Kmax);
bool RSR(double sc_th0, double sc_thf, double sc_Kmax);
bool LSR(double sc_th0, double sc_thf, double sc_Kmax);
bool RSL(double sc_th0, double sc_thf, double sc_Kmax);
bool RLR(double sc_th0, double sc_thf, double sc_Kmax);
bool LRL(double sc_th0, double sc_thf, double sc_Kmax);
DubinsCurve dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

#endif