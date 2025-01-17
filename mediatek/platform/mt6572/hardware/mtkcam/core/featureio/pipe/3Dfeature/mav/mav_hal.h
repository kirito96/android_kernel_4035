

#ifndef _MAV_HAL_H_
#define _MAV_HAL_H_

#include "3DF_hal_base.h"
#include "MTKMav.h"
#include "MTKMotion.h"
#include "MTKWarp.h"


class MTKMav;
class MTKMotion;
class MTKWarp;

class halMAV: public hal3DFBase 
{
public:
    //
    static hal3DFBase* getInstance();
    virtual void destroyInstance();

    /**
     * @brief MAV Hal constructor.
     *
     * @note N/A.
     *
     * @return N/A.
     */
    halMAV(); 

    /**
     * @brief MAV Hal descontrustor.
     *
     * @note N/A.
     *
     * @return N/A.
     */
    virtual ~halMAV();

    /**
     * @brief Init MAV.
     *
     * @note N/A.
     *
     * @param [in] MavInitInData    Input data for MAV algorithm initialization.
     * @param [in] MotionInitInData Input data for Motion algorithm initialization.
     * @param [in] WarpInitInData   Input data for Warping algorithm initialization.
     * @param [in] Pano3DInitInData Input data for Pano3D algorithm initialization.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfInit(void* MavInitInData,void* MotionInitInData,void* WarpInitInData,void* Pano3DInitInData);

    /**
     * @brief Uninit MAV.
     *
     * @note N/A.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfUninit();
    
    /**
     * @brief MAV main function.
     *
     * @note N/A.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHalMavMain();
    
    /**
     * @brief Add input image for MAV.
     *
     * @note N/A.
     *
     * @param [in] pParaIn Input parameters.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfAddImg(MavPipeImageInfo* pParaIn);

    /**
     * @brief Get MAV result by feature control function.
     *
     * @note N/A.
     *
     * @param [out] pParaOut Output parameters.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfGetMavResult(void* pParaOut);  

    /**
     * @brief MAV merge image and return result
     *
     * @note N/A.
     *
     * @param [out] MavResult Output MAV result.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfMerge(MUINT32 *MavResult);

    /**
     * @brief Do motion calculation algorithm.
     *
     * @note N/A.
     *
     * @param [in] InputData Input image address for Motion Algorithm.
     * @param [out] MotionResult Output Motion Result.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfDoMotion(void* InputData, MUINT32* MotionResult, MUINT32 u4SrcImgWidth, MUINT32 u4SrcImgHeight);
    	    
    /**
     * @brief Do warp image.
     *
     * @note N/A.
     *
     * @param [in] pParaIn Input parameters (image address, size, crop offset).
     * @param [in] MavResult MAV result.
     * @param [in] ImgNum Number of warped images.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfWarp(MavPipeImageInfo* pParaIn,MUINT32 *MavResult,MUINT8 ImgNum);
    
    /**
     * @brief Do warp image using clipped size.
     *
     * @note N/A.
     *
     * @param [in] MavResult MAV result.
     * @param [in] ImgNum Number of warped images.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfCrop(MUINT32 *MavResult,MUINT8 ImgNum);
 
    /**
     * @brief Check warp image success or not.
     *
     * @note N/A.
     *
     * @param [out] MavResult A flag to indicate MAV result.
     * @param [out] ClipWidth Clip Width.
     * @param [out] ClipHeight Clip Height.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfGetResult(MUINT32& MavResult,MUINT32& ClipWidth, MUINT32& ClipHeight);  
    	
    /**
     * @brief Get algorithm working buffer size.
     *
     * @note N/A.
     *
     * @param [in] SrcWidth Source image width.
     * @param [in] SrcHeight Source image height.
     * @param [out] WorkingSize Output working buffer size.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfGetWokSize(int SrcWidth, int SrcHeight, MUINT32 &WorkingSize); 	
    
    /**
     * @brief Set algorithm working buffer addr.
     *
     * @note N/A.
     *
     * @param [in] WorkingBuff Input working buffer addr.
     *
     * @return
     * - S_MAV_OK indicates success.
     * - E_MAV_ERR indicates failure.
     * - Other value is error code.
     */
    virtual MINT32 mHal3dfSetWokBuff(void* WorkingBuff);
        
protected:


private:
    MTKMav* m_pMTKMavObj;
    MTKMotion* m_pMTKMotionObj;
    MTKMotion* m_pMTKPanoMotionObj;
    MTKWarp* m_pMTKWarpObj;
    MUINT32* SrcImg;
    MAVMotionResultInfo MAVPreMotionResult;
    MUINT8 FrameCunt;
};

#endif

