



//H263

#define WIDTH_QCIF 176
#define HEIGHT_QCIF 144

#define WIDTH_CIF  352
#define HEIGHT_CIF 288

//������CIF��ʹ��CIF��С������ʹ��QCIF��С
#define CIF			

#ifdef CIF
#define WIDTH		WIDTH_CIF
#define HEIGHT		HEIGHT_CIF
#define PARAM_FORM	CPARAM_CIF
#else
#define WIDTH	WIDTH_QCIF
#define HEIGHT	HEIGHT_QCIF
#define PARAM_FORM	CPARAM_QCIF
#endif

#define VGA_RGB24 1


#define H263_BUFF_SIZE 400000
#define H263_IFRAME_INTERVAL 10