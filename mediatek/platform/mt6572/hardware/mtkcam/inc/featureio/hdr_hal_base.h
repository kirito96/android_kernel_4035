#ifndef _HDR_HAL_BASE_H_
#define _HDR_HAL_BASE_H_

//#include <pipe_types.h>		// For MINT*/MUINT*/M* type names.
#include <mtkcam/common.h>
//#include "MTKHdr.h"



#define HDR_MAX_IMAGE_NUM	3

typedef struct
{
	// For HDR Drv Init.
    MUINT32 u4OutputFrameNum;	// image_num;
    MUINT32 u4FinalGainDiff0;	// ev_gain1;
    MUINT32 u4FinalGainDiff1;	// ev_gain3;
    MUINT32 u4ImgW;				// image_width;
    MUINT32 u4ImgH;				// image_height;
    MUINT32 u4TargetTone;		// target_tone;

    MUINT32 pSourceImgBufAddr[HDR_MAX_IMAGE_NUM];	// image_addr[3]; // input image address

	// For MAV Drv Init.
    MUINT32 pMavWorkingBufAddr;

} HDR_PIPE_INIT_INFO, *P_HDR_PIPE_INIT_INFO;


typedef struct
{
    MUINT32 eHdrRound;  //kidd modify to enum
    MUINT32 u4SourceImgWidth;
    MUINT32 u4SourceImgHeight;
    MUINT32 pSourceImgBufAddr[HDR_MAX_IMAGE_NUM];	// source_image_addr[3]; // source image address
    MUINT32 pSmallImgBufAddr[HDR_MAX_IMAGE_NUM];	// small_image_addr[3]; // small image address
} HDR_PIPE_CONFIG_PARAM, *P_HDR_PIPE_CONFIG_PARAM;

typedef struct
{
    MUINT16 u2SEImgWidth;		// se_image_width;
    MUINT16 u2SEImgHeight;		// se_image_height;
    MUINT32 pSEImgBufAddr[HDR_MAX_IMAGE_NUM];	// se_image_addr;
} HDR_PIPE_SE_INPUT_INFO, *P_HDR_PIPE_SE_INPUT_INFO;

typedef struct
{
    MUINT16	u2SmallImgW;			// Width;	// input image width
    MUINT16	u2SmallImgH;			// Height;	// input image height
    MUINT32	pSmallImgBufAddr[HDR_MAX_IMAGE_NUM];	// ImgAddr;
    MUINT32 pWorkingBuffer;

} HDR_PIPE_FEATURE_EXTRACT_INPUT_INFO, *P_HDR_PIPE_FEATURE_EXTRACT_INPUT_INFO;


typedef struct
{
    MUINT32 bmap_width;
    MUINT32 bmap_height;
    MUINT32 bmap_image_addr[3];
    MUINT32 bmap_image_size;
} HDR_PIPE_SET_BMAP_INFO, *P_HDR_PIPE_SET_BMAP_INFO;

// Come from HDR Drv's WEIGHT_TBL_INFO struc.
typedef struct
{
    MUINT32 weight_table_width;
    MUINT32 weight_table_height;
    MUINT8 *weight_table_data;
} HDR_PIPE_WEIGHT_TBL_INFO, *P_HDR_PIPE_WEIGHT_TBL_INFO;


// Come from HDR Drv's HDR_RESULT_STRUCT struc.
typedef struct
{
    MUINT16 output_image_width;
    MUINT16 output_image_height;
    MUINT32 output_image_addr;
} HDR_PIPE_HDR_RESULT_STRUCT;




class HdrHalBase {
public:
    static HdrHalBase* createInstance();
    virtual void destroyInstance() = 0;
    virtual MBOOL init(void *pInitInData) = 0;
    virtual MBOOL uninit() = 0;
    virtual ~HdrHalBase() {};

public:
    virtual MBOOL Do_Normalization(void) = 0;
    virtual MBOOL Do_SE(HDR_PIPE_SE_INPUT_INFO& rHdrPipeSEInputInfo) = 0;
    virtual MBOOL Do_FeatureExtraction(HDR_PIPE_FEATURE_EXTRACT_INPUT_INFO& rHdrPipeFeatureExtractInputInfo) = 0;
    virtual MBOOL Do_Alignment(void) = 0;
    virtual MBOOL Do_Fusion(HDR_PIPE_WEIGHT_TBL_INFO** pprBlurredWeightMapInfo) = 0;
    virtual MBOOL WeightingMapInfoGet(HDR_PIPE_WEIGHT_TBL_INFO** pprWeightMapInfo) = 0;
    virtual MBOOL WeightingMapInfoSet(HDR_PIPE_SET_BMAP_INFO* pBmapInfo) = 0;
    virtual MBOOL ResultBufferSet(MUINT32 bufferAddr, MUINT32 bufferSize) = 0;
    virtual MBOOL HdrCroppedResultGet(HDR_PIPE_HDR_RESULT_STRUCT& rHdrCroppedResult) = 0;
	virtual MBOOL HdrSmallImgBufSet(HDR_PIPE_CONFIG_PARAM& rHdrPipeConfigParam) = 0;
	virtual MBOOL ConfigMavParam() = 0;
    virtual MBOOL HdrSettingClear(void) = 0;
    virtual MBOOL HdrWorkingBufSet(MUINT32 u4BufAddr, MUINT32 u4BufSize) = 0;
    virtual MUINT32 HdrWorkingBuffSizeGet(void) = 0;
    virtual MBOOL MavWorkingBuffSizeGet(MUINT32 ru4SmallImgWidth, MUINT32 ru4SmallImgHeight, MUINT32 *pMavWorkingBuffSize) = 0;
	virtual void QuerySmallImgResolution(MUINT32& ru4Width, MUINT32& ru4Height) = 0;
    virtual void QuerySEImgResolution(MUINT32& ru4Width, MUINT32& ru4Height) = 0;
    virtual MUINT32 SEImgBuffSizeGet(void) = 0;
    virtual void SaveHdrLog(MUINT32 u4RunningNumber) = 0;
};

#endif	// _HDR_HAL_BASE_H_

