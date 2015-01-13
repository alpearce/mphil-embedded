
#ifndef ADXL345_h
#define ADXL345_h

#define THRESH_TAP_ADDRESS 0x1D            //Tap threshold register
#define OFSX_ADDRESS 0x1E                  //X-axis offset register  
#define OFSY_ADDRESS 0x1F                  //Y-axis offset register
#define OFSZ_ADDRESS 0x20                  //Z-axis offset register
#define DUR_ADDRESS 0x21                   //Tap duration register
#define LATENT_ADDRESS 0x22                //Tap Latency register
#define WINDOW_ADDRESS 0x23                //Tap window register
#define THRESH_ACT_ADDRESS 0x24            //Activity threshold register
#define THRESH_INACT_ADDRESS 0x25          //Inactivity threshold register
#define TIME_INACT_ADDRESS 0x26            //Inactivity time register
#define ACT_INACT_CTL_ADDRESS 0x27         //Axis enable control for activity and inactivity detection register
#define THRESH_FF_ADDRESS 0x28             //Free-fall threshold register
#define TIME_FF_ADDRESS 0x29               //Free-fall time register
#define TAP_AXES_ADDRESS 0x2A              //Axis control for single tap/double tap register 
#define ACT_TAP_STATUS_ADDRESS 0x2B        //Source of single tap/double tap register
#define BW_RATE_ADDRESS 0x2C               //Data rate and power mode control register
#define POWER_CTL_ADDRESS 0x2D             //Power-saving features control register
#define INT_ENABLE_ADDRESS 0x2E            //Interrupt enable control register
#define INT_MAP_ADDRESS 0x2F               //Interrupt mapping control register
#define INT_SOURCE_ADDRESS 0x30            //Source of interrupts register
#define DATA_FORMAT_ADDRESS 0x31           //Data format control register
#define DATAX0_ADDRESS 0x32                //X-Axis Data 0 register                                                      
#define DATAX1_ADDRESS 0x33                //X-Axis Data 1 register
#define DATAY0_ADDRESS 0x34                //Y-Axis Data 0 register
#define DATAY1_ADDRESS 0x35                //Y-Axis Data 1 register
#define DATAZ0_ADDRESS 0x36                //Z-Axis Data 0 register
#define DATAZ1_ADDRESS 0x37                //Z-Axis Data 1 register
#define FIFO_CTL_ADDRESS 0x38              //FIFO control register
#define FIFO_STATUS_ADDRESS 0x39           //FIFO status register
 
#endif

