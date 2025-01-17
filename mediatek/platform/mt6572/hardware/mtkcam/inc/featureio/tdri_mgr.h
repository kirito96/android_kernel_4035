
#ifndef _TDRI_MGR_H_
#define _TDRI_MGR_H_
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
using namespace android;
//-----------------------------------------------------------------------------

typedef enum {
    //// need to re-calculate tpipe table
    TDRI_MGR_FUNC_BNR   = 0,
    TDRI_MGR_FUNC_LSC,
    TDRI_MGR_FUNC_MFB,
    TDRI_MGR_FUNC_CFA,
    TDRI_MGR_FUNC_NBC,
    TDRI_MGR_FUNC_SEEE,
    TDRI_MGR_FUNC_LCE_BASIC,
    TDRI_MGR_FUNC_NR3D_TOP,
    //
    TDRI_MGR_FUNC_NR3D,
    TDRI_MGR_FUNC_LCE_CUSTOM,
    TDRI_MGR_FUNC_OBC,
    TDRI_MGR_FUNC_PGN,
    TDRI_MGR_FUNC_CCL,
    TDRI_MGR_FUNC_G2G,
    TDRI_MGR_FUNC_G2C,
    TDRI_MGR_FUNC_DGM,
    TDRI_MGR_FUNC_GGMRB,
    TDRI_MGR_FUNC_GGMG,
    TDRI_MGR_FUNC_GGM_CTL,
    TDRI_MGR_FUNC_PCA,
    TDRI_MGR_FUNC_PCA_CON,
    TDRI_MGR_FUNC_NUM,
}TDRI_MGR_FUNC_ENUM;






class TdriMgr
{
    public:

    protected:
        TdriMgr() {};
        virtual ~TdriMgr() {};
    //
    public:
        static TdriMgr& getInstance();
        virtual int     init(void) = 0;
        virtual int     uninit(void) = 0;
        //
        virtual MBOOL   flushSetting(ISP_DRV_CQ_ENUM ispCq) = 0;
        virtual MBOOL   applySetting(ISP_DRV_CQ_ENUM ispCq, TDRI_MGR_FUNC_ENUM tmgFunc) = 0;
        //
        virtual MBOOL  setBnr(ISP_DRV_CQ_ENUM ispCq, MBOOL bnrEn, int bpcEn, int bpc_tbl_en, int bpc_tbl_size, int imgciEn, int imgciStride) = 0;
        virtual MBOOL  setLsc(ISP_DRV_CQ_ENUM ispCq, MBOOL lscEn, int sdblk_width, int sdblk_xnum, int sdblk_last_width,
                                    int sdblk_height, int sdblk_ynum, int sdblk_last_height, int lsciEn, int lsciStride) = 0;
        virtual MBOOL  setLce(ISP_DRV_CQ_ENUM ispCq, MBOOL lceEn, int lce_bc_mag_kubnx, int lce_offset_x,
                                    int lce_bias_x, int lce_slm_width, int lce_bc_mag_kubny,
                                    int lce_offset_y, int lce_bias_y, int lce_slm_height, int lceiEn, int lceiStride) = 0;
        virtual MBOOL  setNbc(ISP_DRV_CQ_ENUM ispCq, MBOOL en, int anr_eny,
                            int anr_enc, int anr_iir_mode, int anr_scale_mode) = 0;
        virtual MBOOL  setSeee(ISP_DRV_CQ_ENUM ispCq, MBOOL en, int se_edge, int usm_over_shrink_en) = 0;
        virtual MBOOL  setMfb(ISP_DRV_CQ_ENUM ispCq, int bld_mode, int bld_deblock_en) = 0;
        virtual MBOOL  setCfa(ISP_DRV_CQ_ENUM ispCq, int bayer_bypass) = 0;
        virtual MBOOL  setNr3dTop(ISP_DRV_CQ_ENUM ispCq, MBOOL en) = 0;
        virtual MBOOL  setOtherEngin(ISP_DRV_CQ_ENUM ispCq, TDRI_MGR_FUNC_ENUM engin) = 0;
};


//-----------------------------------------------------------------------------
#endif  // _TDRI_MGR_H_

