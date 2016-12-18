
#ifndef _MTK_CAMERA_INC_CAMPIPE_PARAMS_H_
#define _MTK_CAMERA_INC_CAMPIPE_PARAMS_H_


using namespace NSCamHW; 
namespace NSCamPipe {
////////////////////////////////////////////////////////////////////////////////

 enum ECamPipeNotifyMsg 
{
    ECamPipe_NOTIFY_MSG_NONE          = 0x0000,       /*!< none notify message */    	
    ECamPipe_NOTIFY_MSG_SOF           = 0x0001,       /*!< start of frame notify message */ 
    ECamPipe_NOTIFY_MSG_EOF           = 0x0002,    /*!< end of frame notify  message */
    ECamPipe_NOTIFY_MSG_DROPFRAME     = 0x0004,    /*!< signal of dropped frame */
};

////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamPipe
#endif  //  _MTK_CAMERA_INC_CAMPIPE_PARAMS_H_

