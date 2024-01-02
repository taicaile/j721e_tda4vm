    /* 0, 720p 12bit packed input and output, nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        0x41ce6a26U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 1, 720p 8bit packed input and 12bit packed output,
       nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS8_PACKED, 1280,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        0x41ce6a26U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0x49AA1497, 0x81B8EF6A},
        FALSE, 0x4177B08BDECAFDA1    /* MCRC */
    },
    /* 2, 720p 12bit packed input and 8bit packed output,
       nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        1280, 720, FVID2_CCSF_BITS8_PACKED, 1280,
        0x0u,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 3, 720p 12bit unpacked input and 12bit packed output,
       nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_UNPACKED16, 1280*2,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        0x0u,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 4, 720p 8bit packed input and 8bit packed output,
       nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS8_PACKED, 1280,
        1280, 720, FVID2_CCSF_BITS8_PACKED, 1280,
        0x0u,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0x2221087E, 0x24FFAE1F}, /* LSE PSA */
        FALSE, 0x3752BDB0388F4BEE  /* MCRC */
    },
    /* 5, 1080p 12bit packed input and output, nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1920, 1080, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        1920, 1080, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        0x0U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0x6BDBB59E, 0x29C18A69},
        FALSE, 0x78393B353D7364B9    /* MCRC */
    },
    /* 6, 720p 12bit packed input and output, nf enabled for static filter
       Output is half of input, using even pixel skip */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        640,  360, FVID2_CCSF_BITS12_PACKED, 640*3/2,
        0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 1u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 7, 720p 12bit packed input and output, nf enabled for static filter
       Output is half of input, using odd pixel skip */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_PACKED, 1280*3/2,
        640,  360, FVID2_CCSF_BITS12_PACKED, 640*3/2,
        0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 2u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 8, 720p 12bit unpacked MSB aligned input and output, nf enabled
          for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1280, 720, FVID2_CCSF_BITS12_UNPACKED16_MSB_ALIGNED, 1280*2,
        1280, 720, FVID2_CCSF_BITS12_UNPACKED16_MSB_ALIGNED, 1280*2,
        0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0    /* MCRC */
    },
    /* 9, 8MP 12bit packed input and output, nf enabled
          for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        4096, 2048, FVID2_CCSF_BITS12_PACKED, 4096*3/2,
        4096, 2048, FVID2_CCSF_BITS12_PACKED, 4096*3/2,
        0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0, 0},
        FALSE, 0x61296A25541066B7    /* MCRC */
    },
    /* 10, 1080p 12bit packed input and 8b packed output,
            nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1920, 1080, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        1920, 1080, FVID2_CCSF_BITS8_PACKED, 1920,
        0x0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0xCA4FE297, 0x2A171A66},
        FALSE, 0x0232081EE74C06AB    /* MCRC */
    },
    /* 11, 4MP 12bit packed Luma only input and output, nf enabled
          for static filter */
    {
        FVID2_DF_LUMA_ONLY,
        2048, 2048, FVID2_CCSF_BITS12_PACKED, 2048*3/2,
        2048, 2048, FVID2_CCSF_BITS12_PACKED, 2048*3/2,
        0,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0xEA4272D1, 0},
        FALSE, 0x3F1EE80682E0727D    /* MCRC */
    },
    /* 12, 1080p 12bit packed input and output, nf enabled for static filter */
    {
        FVID2_DF_LUMA_ONLY,
        1920, 1080, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        1920, 1080, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        0x0U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0x6BDBB59E, 0},
        FALSE, 0    /* MCRC */
    },
    /* 13, 1080p 8bit packed input and output, nf enabled for static filter */
    {
        FVID2_DF_YUV420SP_UV,
        1920, 1080, FVID2_CCSF_BITS8_PACKED, 1920,
        1920, 1080, FVID2_CCSF_BITS8_PACKED, 1920,
        0x0U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0xB3A856B8, 0x6D3E78B3},
        FALSE, 0xFC57AF373250615F    /* MCRC */
    },
    /* 14, 1080p 12bit packed input and output, static filter, Chroma only */
    {
        FVID2_DF_CHROMA_ONLY,
        1920, 540, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        1920, 540, FVID2_CCSF_BITS12_PACKED, 1920*3/2,
        0x0U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0x6BDBB59E, 0},
        FALSE, 0    /* MCRC */
    },
    /* 15, 1080p Luma only 8bit packed input and output, nf enabled for static filter */
    {
        FVID2_DF_LUMA_ONLY,
        1920, 1080, FVID2_CCSF_BITS8_PACKED, 1920,
        1920, 1080, FVID2_CCSF_BITS8_PACKED, 1920,
        0x0U,
        NF_FILTER_MODE_BILATERAL,
        255u, 0u, 0u, 0u, 0u, 0u, 0u,
        FALSE, {0xB3A856B8, 0x6D3E78B3},
        FALSE, 0xFC57AF373250615F    /* MCRC */
    },
