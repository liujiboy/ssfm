#ifndef _READPARAMS_H_
#define _READPARAMS_H_

#ifndef FULLQUATSZ
#define FULLQUATSZ     4
#endif


#ifdef __cplusplus

extern "C" {
    
#endif

void readInitialSBAEstimate(char *camsfname, char *ptsfname, int cnp, int pnp, int mnp, 
                                   void (*infilter)(double *pin, int nin, double *pout, int nout), int cnfp,
                                   int *ncams, int *n3Dpts, int *n2Dprojs,
                                   double **motstruct, double **initrot, double **imgpts, double **covimgpts, char **vmask);
void readCalibParams(char *fname, double ical[9]);
int readNumParams(char *fname);

void printSBAMotionData(FILE *fp, double *motstruct, int ncams, int cnp,
                               void (*outfilter)(double *pin, int nin, double *pout, int nout), int cnop);
void printSBAStructureData(FILE *fp, double *motstruct, int ncams, int n3Dpts, int cnp, int pnp);
void printSBAData(FILE *fp, double *motstruct, int cnp, int pnp, int mnp, 
                         void (*outfilter)(double *pin, int nin, double *pout, int nout), int cnop,
                         int ncams, int n3Dpts, double *imgpts, int n2Dprojs, char *vmask);

void saveSBAStructureDataAsPLY(char *fname, double *motstruct, int ncams, int n3Dpts, int cnp, int pnp, int withrgb);
#ifdef __cplusplus
}
#endif
#endif /* _READPARAMS_H_ */
