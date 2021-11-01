/************************************************************************
* Copyright (C) 2012-2018, Focaltech Systems (R),All Rights Reserved.
*
* File Name: focaltech_test_ft8201.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-08-08
*
* Abstract: test item for ft8201
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include "../focaltech_test.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define REG_GIP_SD_OPT          0x20
#define REG_VERF                0x86
#define REG_MS_SELECT           0x26
#define REG_CH_X_MASTER         0x50
#define REG_CH_Y_MASTER         0x51
#define REG_CH_X_SLAVE          0x52
#define REG_CH_Y_SLAVE          0x53
#define REG_FW_INFO_CNT         0x17
#define I2C_ADDR_M              0
#define I2C_ADDR_S              12
#define REG_FW_INFO_ADDR        0x81
#define REG_FW_INFO_LEN         32

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum M_S_TYPE {
    CHIP_AS_SLAVE = 0,
    CHIP_AS_MASTER = 1,
    SINGLE_CHIP = 3,
};

enum CASCADE_DIRECTION {
    CASCADE_LEFT_RIGHT = 0,
    CASCADE_UP_DOWN    = 1,
};

/*
 * m_s_sel    - master/slave information
 * m_i2c_addr - master ic I2C address
 * s_i2c_addr - slave ic I2C address
 * m_tx       - master IC tx number
 * m_rx       - master IC rx number
 * s_tx       - slave IC tx number
 * s_rx       - slave IC rx number
 */
struct ft8201_info {
    union m_s_sel {
        struct bits {
            u8 type         : 6;
            u8 direction    : 1;
            u8 s0_as_slave  : 1;
        } bits;
        u8 byte_val;
    } m_s_sel;
    u8 m_i2c_addr;
    u8 s_i2c_addr;
    u8 m_tx;
    u8 m_rx;
    u8 s_tx;
    u8 s_rx;
    u8  current_slave_addr;
};

struct ft8201_test_item {
    bool rawdata_test;
    bool cb_test;
    bool short_test;
    bool lcd_noise_test;
    bool open_test;
};
struct ft8201_basic_threshold {
    int rawdata_test_min;
    int rawdata_test_max;
    bool cb_test_va_check;
    int cb_test_min;
    int cb_test_max;
    bool cb_test_vk_check;
    int cb_test_min_vk;
    int cb_test_max_vk;
    int short_res_min;
    int short_res_vk_min;
    int lcd_noise_test_frame;
    int lcd_noise_test_max_screen;
    int lcd_noise_test_max_frame;
    int lcd_noise_coefficient;
    int lcd_noise_coefficient_key;
    int open_test_min;
};

enum test_item_ft8201 {
    CODE_FT8201_ENTER_FACTORY_MODE = 0,
    CODE_FT8201_RAWDATA_TEST = 7,
    CODE_FT8201_CHANNEL_NUM_TEST = 8,
    CODE_FT8201_CB_TEST = 12,
    CODE_FT8201_SHORT_CIRCUIT_TEST = 14,
    CODE_FT8201_LCD_NOISE_TEST = 15,
    CODE_FT8201_OPEN_TEST = 25,
};

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct ft8201_test_item ft8201_item;
struct ft8201_basic_threshold ft8201_basic_thr;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static void fts_array_copy(int *dest, const int *src, int len)
{
    int i = 0;

    for (i = 0; i < len; i++) {
        dest[i] = src[i];
    }
}

static void work_as_master(struct ft8201_info *info)
{
    if (fts_data->client->addr != info->m_i2c_addr) {
        FTS_TEST_DBG("change i2c addr to master(0x%x)\n", info->m_i2c_addr);
        fts_data->client->addr = info->m_i2c_addr;
    }
}

static void work_as_slave(struct ft8201_info *info)
{
    if (fts_data->client->addr != info->s_i2c_addr) {
        FTS_TEST_DBG("change i2c addr to slave(0x%x)\n", info->s_i2c_addr);
        fts_data->client->addr = info->s_i2c_addr;
    }
}

static int ft8201_write_reg(struct ft8201_info *info, u8 reg_addr, u8 reg_val)
{
    int ret = 0;

    /* write master reg */
    work_as_master(info);
    ret = write_reg(reg_addr, reg_val);
    if (ret) {
        FTS_TEST_SAVE_ERR("write master reg fail\n");
        return ret;
    }

    /* write slave reg */
    work_as_slave(info);
    ret = write_reg(reg_addr, reg_val);
    if (ret) {
        FTS_TEST_SAVE_ERR("write slave reg fail\n");
        work_as_master(info);
        return ret;
    }
    work_as_master(info);

    return 0;
}

static void integrate_data(struct ft8201_info *info, int *m_buf, int *s_buf, int *data)
{
    int i = 0;
    int *s0_buf;
    int *s1_buf;
    int s0_ch = 0;
    int s0_tx = 0;
    int s0_rx = 0;
    int s1_ch = 0;
    int s1_rx = 0;
    int row = 0;
    int s0_row = 0;
    int s1_row = 0;

    FTS_TEST_FUNC_ENTER();

    if (false == info->m_s_sel.bits.s0_as_slave) {
        s0_buf = m_buf;
        s0_tx = info->m_tx;
        s0_rx = info->m_rx;
        s0_ch = info->m_tx * info->m_rx;
        s1_buf = s_buf;
        s1_rx = info->s_rx;
        s1_ch = info->s_tx * info->s_rx;
    } else {
        s0_buf = s_buf;
        s0_tx = info->s_tx;
        s0_rx = info->s_rx;
        s0_ch = info->s_tx * info->s_rx;
        s1_buf = m_buf;
        s1_rx = info->m_rx;
        s1_ch = info->m_tx * info->m_rx;
    }

    FTS_TEST_DBG("%d %d %d %d %d", s0_tx, s0_rx, s0_ch, s1_rx, s1_ch);
    if (CASCADE_LEFT_RIGHT == info->m_s_sel.bits.direction) {
        /* cascade direction : left to right */
        for (i = 0; i < s0_tx; i++) {
            row = i * (s0_rx + s1_rx);
            s0_row = i * s0_rx;
            s1_row = i * s1_rx;

            fts_array_copy(data + row, s0_buf + s0_row, s0_rx);
            fts_array_copy(data + row + s0_rx, s1_buf + s1_row, s1_rx);
        }

    } else {
        /* cascade direction : up to down */
        fts_array_copy(data, s0_buf, s0_ch);
        fts_array_copy(data + s0_ch, s1_buf, s1_ch);
    }

    /* key */
    fts_array_copy(data + s0_ch + s1_ch, s0_buf + s0_ch, 6);
    fts_array_copy(data + s0_ch + s1_ch + 6, s1_buf + s1_ch, 6);

    FTS_TEST_FUNC_EXIT();
}

static int ft8201_get_key_num(void)
{
    int ret = 0;
    int i = 0;
    u8 keyval = 0;

    test_data.screen_param.key_num = 0;
    for (i = 0; i < 3; i++) {
        ret = read_reg( FACTORY_REG_LEFT_KEY, &keyval );
        if (0 == ret) {
            if (((keyval >> 0) & 0x01)) {
                test_data.screen_param.left_key1 = true;
                ++test_data.screen_param.key_num;
            }
            if (((keyval >> 1) & 0x01)) {
                test_data.screen_param.left_key2 = true;
                ++test_data.screen_param.key_num;
            }
            if (((keyval >> 2) & 0x01)) {
                test_data.screen_param.left_key3 = true;
                ++test_data.screen_param.key_num;
            }
            break;
        } else {
            sys_delay(150);
            continue;
        }
    }

    if (i >= 3) {
        FTS_TEST_SAVE_ERR("can't get left key num");
        return ret;
    }

    for (i = 0; i < 3; i++) {
        ret = read_reg( FACTORY_REG_RIGHT_KEY, &keyval );
        if (0 == ret) {
            if (((keyval >> 0) & 0x01)) {
                test_data.screen_param.right_key1 = true;
                ++test_data.screen_param.key_num;
            }
            if (((keyval >> 1) & 0x01)) {
                test_data.screen_param.right_key2 = true;
                ++test_data.screen_param.key_num;
            }
            if (((keyval >> 2) & 0x01)) {
                test_data.screen_param.right_key3 = true;
                ++test_data.screen_param.key_num;
            }
            break;
        } else {
            sys_delay(150);
            continue;
        }
    }

    if (i >= 3) {
        FTS_TEST_SAVE_ERR("can't get right key num");
        return ret;
    }

    return 0;
}

static int check_ic_info_validity(struct ft8201_info *info)
{
    /* IC type */
    if ((info->m_s_sel.bits.type != CHIP_AS_SLAVE)
        && (info->m_s_sel.bits.type != CHIP_AS_MASTER)) {
        FTS_TEST_SAVE_ERR("IC cascade type(%d) fail\n", info->m_s_sel.bits.type);
        return -EINVAL;
    }

    /* I2C addr */
    if ((0 == info->m_i2c_addr) || (0 == info->s_i2c_addr)) {
        FTS_TEST_SAVE_ERR("i2c addr of master(0x%x)/slave(0x%x) fail\n",
                          info->m_i2c_addr, info->s_i2c_addr);
        return -EINVAL;
    }

    /* tx/rx */
    if ((0 == info->m_tx) || (info->m_tx > TX_NUM_MAX)) {
        FTS_TEST_SAVE_ERR("master tx(%d) fail\n", info->m_tx);
        return -EINVAL;
    }

    if ((0 == info->m_rx) || (info->m_rx > TX_NUM_MAX)) {
        FTS_TEST_SAVE_ERR("master rx(%d) fail\n", info->m_rx);
        return -EINVAL;
    }

    if ((0 == info->s_tx) || (info->s_tx > TX_NUM_MAX)) {
        FTS_TEST_SAVE_ERR("slave tx(%d) fail\n", info->s_tx);
        return -EINVAL;
    }

    if ((0 == info->s_rx) || (info->s_rx > TX_NUM_MAX)) {
        FTS_TEST_SAVE_ERR("slave rx(%d) fail\n", info->s_rx);
        return -EINVAL;
    }

    return 0;
}

static int get_chip_information(struct ft8201_info *info)
{
    int ret = 0;
    u8 value[REG_FW_INFO_LEN] = { 0 };
    u8 cmd = 0;

    ret = read_reg(REG_MS_SELECT, &value[0]);
    if (ret) {
        FTS_TEST_SAVE_ERR("read m/s select info fail\n");
        return ret;
    }
    info->m_s_sel.byte_val = value[0];

    ret = read_reg(REG_CH_X_MASTER, &value[0]);
    if (ret) {
        FTS_TEST_SAVE_ERR("read ch_x_m fail\n");
        return ret;
    }
    info->m_tx = value[0];

    ret = read_reg(REG_CH_Y_MASTER, &value[0]);
    if (ret) {
        FTS_TEST_SAVE_ERR("read ch_y_m fail\n");
        return ret;
    }
    info->m_rx = value[0];

    ret = read_reg(REG_CH_X_SLAVE, &value[0]);
    if (ret) {
        FTS_TEST_SAVE_ERR("read ch_x_s fail\n");
        return ret;
    }
    info->s_tx = value[0];

    ret = read_reg(REG_CH_Y_SLAVE, &value[0]);
    if (ret) {
        FTS_TEST_SAVE_ERR("read ch_y_s fail\n");
        return ret;
    }
    info->s_rx = value[0];

    ret = write_reg(REG_FW_INFO_CNT, 0);
    if (ret) {
        FTS_TEST_SAVE_ERR("write fw into cnt fail\n");
        return ret;
    }
    cmd = REG_FW_INFO_ADDR;
    ret = fts_test_i2c_read(&cmd, 1, &value[0], REG_FW_INFO_LEN);
    if (ret) {
        FTS_TEST_SAVE_ERR("read fw info fail\n");
        return ret;
    }

    if ((value[I2C_ADDR_M] + value[I2C_ADDR_M + 1]) == 0xFF) {
        info->m_i2c_addr = value[I2C_ADDR_M] >> 1;
    }

    if ((value[I2C_ADDR_S] + value[I2C_ADDR_S + 1]) == 0xFF) {
        info->s_i2c_addr = value[I2C_ADDR_S] >> 1;
    }

    FTS_TEST_DBG("%s=%d,%s=%d,%s=%d,%s=0x%x,%s=0x%x,%s=%d,%s=%d,%s=%d,%s=%d\n",
                 "type", info->m_s_sel.bits.type,
                 "direction", info->m_s_sel.bits.direction,
                 "s0_as_slave", info->m_s_sel.bits.s0_as_slave,
                 "m_i2c_addr", info->m_i2c_addr,
                 "s_i2c_addr", info->s_i2c_addr,
                 "m_tx", info->m_tx,
                 "m_rx", info->m_rx,
                 "s_tx", info->s_tx,
                 "s_rx", info->s_rx
                );

    ret = check_ic_info_validity(info);
    if (ret) {
        FTS_TEST_SAVE_ERR("ic information invalid\n");
        return ret;
    }

    return 0;
}

static int ft8201_test_init(struct ft8201_info *info)
{
    int ret = 0;

    /* initialize info */
    memset(info, 0, sizeof(struct ft8201_info));
    test_data.screen_param.key_num_total = KEY_NUM_MAX * 2;

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail, can't get tx/rx num\n");
        return ret;
    }

    /* get chip info */
    ret = get_chip_information(info);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get chip information fail\n");
        return ret;
    }

    return 0;
}

static int ft8201_enter_factory_mode(struct ft8201_info *info)
{
    int ret = 0;

    ret = enter_factory_mode();
    if (ret) {
        FTS_TEST_SAVE_ERR("enter factory mode fail, can't get tx/rx num\n");
        return ret;
    }

    ret = ft8201_get_key_num();
    if (ret) {
        FTS_TEST_SAVE_ERR("get key num fail\n");
        return ret;
    }

    return 0;
}

static u8 ft8201_chip_clb(struct ft8201_info *info)
{
    int ret = 0;

    FTS_TEST_FUNC_ENTER();
    /* master clb */
    work_as_master(info);
    ret = chip_clb_incell();
    if (ret) {
        FTS_TEST_SAVE_ERR("master clb fail\n");
        return ret;
    }

    /* slave clb */
    work_as_slave(info);
    ret = chip_clb_incell();
    if (ret) {
        FTS_TEST_SAVE_ERR("master clb fail\n");
        work_as_master(info);
        return ret;
    }
    work_as_master(info);

    FTS_TEST_FUNC_EXIT();
    return 0;
}

static int ft8201_get_tx_rx_cb(struct ft8201_info *info, u8 start_node, int read_num, int *read_buffer)
{
    int ret = 0;
    int *buffer_master = NULL;
    int *buffer_slave = NULL;
    int master_tx = info->m_tx;
    int master_rx = info->m_rx;
    int slave_tx = info->s_tx;
    int slave_rx = info->s_rx;

    FTS_TEST_FUNC_ENTER();

    buffer_master = fts_malloc((master_tx + 1) * master_rx * sizeof(int));
    if (NULL == buffer_master) {
        FTS_TEST_SAVE_ERR("%s:master buf malloc fail\n", __func__);
        ret = -ENOMEM;
        goto GET_CB_ERR;
    }

    buffer_slave = fts_malloc((slave_tx + 1) * slave_rx * sizeof(int));
    if (NULL == buffer_slave) {
        FTS_TEST_SAVE_ERR("%s:slave buf malloc fail\n", __func__);
        ret = -ENOMEM;
        goto GET_CB_ERR;
    }

    /* master cb */
    work_as_master(info);
    ret = get_cb_incell(0, master_tx * master_rx  + 6, buffer_master);
    if (ret ) {
        FTS_TEST_SAVE_ERR("master clb fail\n");
        goto GET_CB_ERR;
    }

    /* slave cb */
    work_as_slave(info);
    ret = get_cb_incell(0, slave_tx * slave_rx + 6, buffer_slave);
    if (ret ) {
        FTS_TEST_SAVE_ERR("slave clb fail\n");
        work_as_master(info);
        goto GET_CB_ERR;
    }
    work_as_master(info);

    integrate_data(info, buffer_master, buffer_slave, read_buffer);

GET_CB_ERR:
    fts_free(buffer_master);
    fts_free(buffer_slave);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int read_adc_data(u8 retval, int byte_num, int *adc_buf)
{
    int ret = 0;
    int times = 0;
    u8 short_state = 0;

    FTS_TEST_FUNC_ENTER();

    for (times = 0; times < FACTORY_TEST_RETRY; times++) {
        ret = read_reg(FACTORY_REG_SHORT_TEST_STATE, &short_state);
        if ((0 == ret) && (retval == short_state))
            break;
        else
            FTS_TEST_DBG("reg%x=%x,retry:%d",
                         FACTORY_REG_SHORT_TEST_STATE, short_state, times);

        sys_delay(FACTORY_TEST_RETRY_DELAY);
    }
    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("short test timeout, ADC data not OK\n");
        ret = -EIO;
        goto ADC_ERROR;
    }

    ret = read_mass_data(FACTORY_REG_SHORT_ADDR, byte_num, adc_buf);
    if (ret) {
        FTS_TEST_SAVE_ERR("get short(adc) data fail\n");
    }

ADC_ERROR:
    FTS_TEST_FUNC_EXIT();
    return ret;
}

static u8 ft8201_weakshort_get_adcdata(struct ft8201_info *info, int *rbuf)
{
    int ret = 0;
    int master_adc_num = 0;
    int slave_adc_num = 0;
    int *buffer_master = NULL;
    int *buffer_slave = NULL;
    int master_tx = info->m_tx;
    int master_rx = info->m_rx;
    int slave_tx = info->s_tx;
    int slave_rx = info->s_rx;
    int ch_num = 0;

    FTS_TEST_FUNC_ENTER();

    buffer_master = fts_malloc((master_tx + 1) * master_rx * sizeof(int));
    if (NULL == buffer_master) {
        FTS_TEST_SAVE_ERR("%s:master buf malloc fail\n", __func__);
        ret = -ENOMEM;
        goto ADC_ERROR;
    }

    buffer_slave = fts_malloc((slave_tx + 1) * slave_rx * sizeof(int));
    if (NULL == buffer_slave) {
        FTS_TEST_SAVE_ERR("%s:slave buf malloc fail\n", __func__);
        ret = -ENOMEM;
        goto ADC_ERROR;
    }

    /* Start ADC sample */
    ch_num = master_tx + master_rx;
    ret = write_reg(FACTORY_REG_SHORT_TEST_EN, 0x01);
    if (ret) {
        FTS_TEST_SAVE_ERR("start short test fail\n");
        goto ADC_ERROR;
    }
    sys_delay(ch_num * FACTORY_TEST_DELAY);

    /* read master adc data */
    master_adc_num = (master_tx * master_rx + 6) * 2;
    work_as_master(info);
    ret = read_adc_data(TEST_RETVAL_00, master_adc_num, buffer_master);
    if (ret) {
        FTS_TEST_SAVE_ERR("read master adc data fail\n");
        goto ADC_ERROR;
    }

    /* read slave adc data */
    slave_adc_num = (slave_tx * slave_rx + 6) * 2;
    work_as_slave(info);
    ret = read_adc_data(TEST_RETVAL_00, slave_adc_num, buffer_slave);
    if (ret) {
        FTS_TEST_SAVE_ERR("read master adc data fail\n");
        work_as_master(info);
        goto ADC_ERROR;
    }
    work_as_master(info);

    /* data integration */
    integrate_data(info, buffer_master, buffer_slave, rbuf);

ADC_ERROR:
    fts_free(buffer_master);
    fts_free(buffer_slave);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft8201_short_test(struct ft8201_info *info, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    u8 tx_num = 0;
    u8 rx_num = 0;
    u8 key_num = 0;
    int row = 0;
    int col = 0;
    int tmp_adc = 0;
    int value_min = 0;
    int vk_value_min = 0;
    int value_max = 0;
    int *adcdata = NULL;

    FTS_TEST_SAVE_INFO("\r\n\r\n==============================Test Item: -------- Short Circuit Test\r\n");
    tx_num = test_data.screen_param.tx_num;
    rx_num = test_data.screen_param.rx_num;
    key_num = test_data.screen_param.key_num_total;
    memset(test_data.buffer, 0, ((tx_num + 1) * rx_num) * sizeof(int));
    adcdata = test_data.buffer;

    ret = enter_factory_mode();
    if (ret) {
        FTS_TEST_SAVE_ERR("//Failed to Enter factory mode.ret=%d\n", ret);
        goto TEST_END;
    }

    ret = ft8201_weakshort_get_adcdata(info, adcdata);
    if (ret) {
        FTS_TEST_SAVE_ERR("//Failed to get AdcData. ret=%d\n", ret);
        goto TEST_END;
    }

    /* show ADCData */
#if 0
    FTS_TEST_SAVE_INFO("ADCData:\n");
    for (row = 0; row < byte_num / 2; row++) {
        FTS_TEST_SAVE_INFO("%-4d  ", adcdata[row]);
        if (0 == (row + 1) % rx_num) {
            FTS_TEST_SAVE_INFO("\n");
        }
    }
    FTS_TEST_SAVE_INFO("\n");
#endif

    /* calculate resistor */
    for (row = 0; row < tx_num + 1; row++) {
        for (col = 0; col < rx_num; col++) {
            tmp_adc = adcdata[row * rx_num + col];
            if (tmp_adc > 4050)  tmp_adc = 4050;
            adcdata[row * rx_num + col] = (tmp_adc * 100) / (4095 - tmp_adc);
        }
    }

    show_data_incell(adcdata, true);

    /* analyze data */
    value_min = ft8201_basic_thr.short_res_min;
    vk_value_min = ft8201_basic_thr.short_res_vk_min;
    value_max = 100000000;
    FTS_TEST_SAVE_INFO("Short Circuit test , VA_Set_Range=(%d, %d), VK_Set_Range=(%d, %d)\n", \
                       value_min, value_max, vk_value_min, value_max);

    tmp_result = compare_data_incell(adcdata, value_min, value_max, vk_value_min, value_max, true);

    save_testdata_incell(adcdata, "Short Circuit Test", 0, tx_num + 1, rx_num, 1);

TEST_END:
    if (tmp_result) {
        FTS_TEST_SAVE_INFO("//Short Circuit Test is OK!\n");
        *test_result = true;
    } else {
        FTS_TEST_SAVE_INFO("//Short Circuit Test is NG!\n");
        *test_result = false;
    }

    return ret;
}

static int ft8201_open_test(struct ft8201_info *info, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    u8 tx_num = 0;
    u8 rx_num = 0;
    int byte_num = 0;
    u8 ch_value = 0xFF;
    u8 reg_data = 0xFF;
    u8 tmp_val = 0;
    int max_value = 0;
    int min_value = 0;
    int *opendata = NULL;

    FTS_TEST_SAVE_INFO("\r\n\r\n==============================Test Item: --------  Open Test \n");
    tx_num = test_data.screen_param.tx_num;
    rx_num = test_data.screen_param.rx_num;
    memset(test_data.buffer, 0, ((tx_num + 1) * rx_num) * sizeof(int));
    opendata = test_data.buffer;

    ret = enter_factory_mode();
    if (ret) {
        FTS_TEST_SAVE_ERR("Enter Factory Failed\n");
        goto TEST_END;
    }

    ret = read_reg(REG_GIP_SD_OPT, &ch_value);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to Read Reg\n ");
        goto TEST_END;
    }

    ret = read_reg(REG_VERF, &reg_data);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to Read Reg\n ");
        goto TEST_END;
    }

    /* set VREF_TP to 2V */
    ret = ft8201_write_reg(info, REG_VERF, 0x01);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to Read or Write Reg\n ");
        goto TEST_END;
    }

    /* set Bit4~Bit5 of reg0x20 is set to 2b'10 (Source to GND) */
    tmp_val = reg_data | (1 << 5);
    tmp_val &= ~(1 << 4);
    ret = ft8201_write_reg(info, REG_GIP_SD_OPT, tmp_val);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to Read or Write Reg\n");
        goto TEST_END;
    }

    /* wait fw state update before clb */
    wait_state_update();

    ret = ft8201_chip_clb(info);
    if (ret) {
        FTS_TEST_SAVE_ERR("//========= auto clb Failed\n");
        goto TEST_END;
    }

    /* get cb data */
    byte_num = tx_num * rx_num  + test_data.screen_param.key_num_total;
    ret = ft8201_get_tx_rx_cb(info, 0, byte_num, opendata);
    if (ret) {
        FTS_TEST_SAVE_ERR("\r\n\r\n//=========get CB Failed!");
        goto TEST_END;
    }

    /* show open data */
    show_data_incell(opendata, false);

    /* compare */
    min_value = ft8201_basic_thr.open_test_min;
    max_value = 255;
    tmp_result = compare_data_incell(opendata, min_value, max_value, 0, 0, false);

    /* save data */
    save_testdata_incell( opendata, "Open Test", 0, tx_num, rx_num, 1 );

    /* restore register */
    ret = ft8201_write_reg(info, REG_GIP_SD_OPT, ch_value);
    if (ret) {
        FTS_TEST_SAVE_ERR("\r\nFailed to Read or Write Reg!\r\n");
        tmp_result = false;
        goto TEST_END;
    }

    ret = ft8201_write_reg(info, REG_VERF, reg_data);
    if (ret) {
        FTS_TEST_SAVE_ERR("\r\nFailed to Read or Write Reg!\r\n");
        tmp_result = false;
        goto TEST_END;
    }

    /* wait fw state update before clb */
    wait_state_update();

    ret = ft8201_chip_clb(info);
    if (ret) {
        FTS_TEST_SAVE_ERR("//========= auto clb Failed\n");
        tmp_result = false;
        goto TEST_END;
    }

TEST_END:
    if (tmp_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("//Open Test is OK!\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_INFO("//Open Test is NG!\n");
    }

    return ret;
}

static int ft8201_cb_test(struct ft8201_info *info, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    u8 tx_num = 0;
    u8 rx_num = 0;
    int byte_num = 0;
    bool include_key = false;
    int *cbdata = NULL;

    FTS_TEST_SAVE_INFO("\n\n==============================Test Item: --------  CB Test\n");
    tx_num = test_data.screen_param.tx_num;
    rx_num = test_data.screen_param.rx_num;
    include_key = ft8201_basic_thr.cb_test_vk_check;
    memset(test_data.buffer, 0, ((tx_num + 1) * rx_num) * sizeof(int));
    cbdata = test_data.buffer;

    ret = enter_factory_mode();
    if (ret) {
        FTS_TEST_SAVE_ERR("// Failed to Enter factory mode.ret:%d\n", ret);
        goto TEST_ERR;
    }

    ret = ft8201_chip_clb(info);
    if (ret) {
        FTS_TEST_SAVE_ERR("//========= auto clb Failed\n");
        goto TEST_ERR;
    }

    byte_num = tx_num * rx_num  + test_data.screen_param.key_num_total;
    ret = ft8201_get_tx_rx_cb(info, 0, byte_num, cbdata);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to get CB value...\n");
        goto TEST_ERR;
    }

    /* Show CbData */
    show_data_incell(cbdata, include_key);

    /* compare data */
    tmp_result = compare_detailthreshold_data_incell(cbdata,
                 test_data.incell_detail_thr.cb_test_min,
                 test_data.incell_detail_thr.cb_test_max,
                 include_key);

    /* Save Test Data */
    save_testdata_incell(cbdata, "CB Test", 0, tx_num + 1, rx_num, 1);

TEST_ERR:
    if (tmp_result) {
        FTS_TEST_SAVE_INFO("//CB Test is OK!\n");
        *test_result = true;
    } else {
        FTS_TEST_SAVE_INFO("//CB Test is NG!\n");
        *test_result = false;
    }

    return ret;
}

static int ft8201_rawdata_test(struct ft8201_info *info, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    u8 tx_num = 0;
    u8 rx_num = 0;
    int i = 0;
    int *rawdata = NULL;

    FTS_TEST_SAVE_INFO("\n\n==============================Test Item: -------- Raw Data Test\n");
    tx_num = test_data.screen_param.tx_num;
    rx_num = test_data.screen_param.rx_num;
    memset(test_data.buffer, 0, ((tx_num + 1) * rx_num) * sizeof(int));
    rawdata = test_data.buffer;

    ret = enter_factory_mode();
    if (ret) {
        FTS_TEST_SAVE_ERR("//Failed to Enter factory mode.ret:%d", ret);
        return ret;
    }

    /* Read RawData */
    for (i = 0 ; i < 3; i++) {
        ret = get_rawdata_incell(rawdata);
    }
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to get Raw Data,ret:%d", ret);
        return ret;
    }

    /* Show RawData */
    show_data_incell(rawdata, true);

    /* To Determine RawData if in Range or not */
    tmp_result = compare_detailthreshold_data_incell(rawdata, test_data.incell_detail_thr.rawdata_test_min, test_data.incell_detail_thr.rawdata_test_max, true);

    /* Save Test Data */
    save_testdata_incell(rawdata, "RawData Test", 0, tx_num + 1, rx_num, 1);

    /* Return Result */
    if (tmp_result) {
        FTS_TEST_SAVE_INFO("//RawData Test is OK\n");
        *test_result = true;
    } else {
        FTS_TEST_SAVE_INFO("//RawData Test is NG\n");
        *test_result = false;
    }

    return ret;
}

static int ft8201_lcdnoise_test(struct ft8201_info *info, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    u8 tx_num = 0;
    u8 rx_num = 0;
    int key_num = 0;
    int lcdnoise_bytenum = 0;
    u8 frame_num = 0;
    int retry = 0;
    u8 status = 0xFF;
    int row = 0;
    int col = 0;
    int value_min = 0;
    int value_max = 0;
    int vk_value_max = 0;
    int *lcdnoise = NULL;

    FTS_TEST_SAVE_INFO("\r\n\r\n==============================Test Item: -------- LCD Noise Test \r\n");
    tx_num = test_data.screen_param.tx_num;
    rx_num = test_data.screen_param.rx_num;
    key_num = test_data.screen_param.key_num_total;
    memset(test_data.buffer, 0, ((tx_num + 1) * rx_num) * sizeof(int));
    lcdnoise = test_data.buffer;

    ret = enter_factory_mode();
    if (ret) {
        tmp_result = false;
        FTS_TEST_SAVE_ERR("//Failed to Enter factory mode.ret:%d", ret);
        goto TEST_END;
    }

    ret = ft8201_write_reg(info, FACTORY_REG_DATA_SELECT, 0x01);
    if (ret) {
        FTS_TEST_SAVE_ERR("write data select reg fail\n");
        goto TEST_END;
    }

    /* set scan number */
    frame_num = ft8201_basic_thr.lcd_noise_test_frame;
    ret = ft8201_write_reg(info, FACTORY_REG_LCD_NOISE_FRAME, frame_num & 0xFF );
    if (ret) {
        FTS_TEST_SAVE_ERR("write lcd noise low reg fail\n");
        goto TEST_END;
    }
    ret = ft8201_write_reg(info, FACTORY_REG_LCD_NOISE_FRAME + 1, (frame_num >> 8) & 0xFF);
    if (ret) {
        FTS_TEST_SAVE_ERR("write lcd noise high reg fail\n");
        goto TEST_END;
    }

    /* set point */
    ret = ft8201_write_reg(info, FACTORY_REG_LINE_ADDR, 0xAD);
    if (ret) {
        FTS_TEST_SAVE_ERR("write line addr reg fail\n");
        goto TEST_END;
    }

    /* start lcd noise test */
    ret = write_reg(FACTORY_REG_LCD_NOISE_START, 0x01);
    if (ret) {
        FTS_TEST_SAVE_ERR("write lcd noise start reg fail\n");
        goto TEST_END;
    }

    /* check status */
    sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
    for (retry = 0; retry < FACTORY_TEST_RETRY; retry++) {
        status = 0xFF;
        ret = read_reg(FACTORY_REG_LCD_NOISE_START, &status );
        if ((0 == ret) && (TEST_RETVAL_00 == status)) {
            break;
        } else {
            FTS_TEST_DBG("reg%x=%x,retry:%d\n",
                         FACTORY_REG_LCD_NOISE_START, status, retry);
        }
        sys_delay(FACTORY_TEST_RETRY_DELAY);
    }
    if (retry >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_INFO("Scan Noise Time Out!");
        goto TEST_END;
    }

    lcdnoise_bytenum = tx_num * rx_num * 2 + key_num * 2;
    ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR, lcdnoise_bytenum, lcdnoise);
    if (ret) {
        FTS_TEST_SAVE_ERR("Failed to Read Data.\n");
        goto TEST_END;
    }

    for (row = 0; row < tx_num + 1; row++) {
        for (col = 0; col < rx_num; col++) {
            lcdnoise[row * rx_num + col] = focal_abs( lcdnoise[row * rx_num + col]);
        }
    }

    /* show lcd noise data */
    show_data_incell(lcdnoise, true);

    /* compare */
    value_min = 0;
    value_max = ft8201_basic_thr.lcd_noise_coefficient * test_data.va_touch_thr * 32 / 100;
    vk_value_max = ft8201_basic_thr.lcd_noise_coefficient_key * test_data.key_touch_thr * 32 / 100;
    tmp_result = compare_data_incell(lcdnoise, value_min, value_max, value_min, vk_value_max, true);

    /* save data */
    save_testdata_incell(lcdnoise, "LCD Noise Test", 0, tx_num + 1, rx_num, 1);

TEST_END:
    ret = ft8201_write_reg(info, FACTORY_REG_DATA_SELECT, 0x00);
    if (ret) {
        FTS_TEST_SAVE_ERR("write data select reg fail\n");
        tmp_result = false;
    }

    ret = write_reg(FACTORY_REG_LCD_NOISE_START, 0x00);
    if (ret) {
        FTS_TEST_SAVE_ERR("write 0 to lcd noise reg fail\n");
        tmp_result = false;
    }

    if (tmp_result) {
        FTS_TEST_SAVE_INFO("//LCD Noise Test is OK!\n");
        *test_result = true;
    } else {
        FTS_TEST_SAVE_INFO("//LCD Noise Test is NG!\n");
        *test_result = false;
    }

    return ret;
}


static bool start_test_ft8201(void)
{
    int ret = 0;
    bool test_result = true;
    bool tmp_result = true;
    u8 item_code = 0;
    int item_count = 0;
    struct ft8201_info info;

    FTS_TEST_FUNC_ENTER();

    if (0 == test_data.test_num) {
        FTS_TEST_SAVE_ERR("test item == 0\n");
        return false;
    }

    ret = ft8201_test_init(&info);
    if (ret) {
        FTS_TEST_SAVE_ERR("test init fail\n");
        return ret;
    }

    for (item_count = 0; item_count < test_data.test_num; item_count++) {
        item_code = test_data.test_item[item_count].itemcode;
        test_data.test_item_code = item_code;

        /* ENTER_FACTORY_MODE */
        if (CODE_FT8201_ENTER_FACTORY_MODE == item_code) {
            ret = ft8201_enter_factory_mode(&info);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
                break;  /* if this item FAIL, no longer test. */
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

        /* SHORT_CIRCUIT_TEST */
        if (CODE_FT8201_SHORT_CIRCUIT_TEST == item_code) {
            ret = ft8201_short_test(&info, &tmp_result);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

        /* OPEN_TEST */
        if (CODE_FT8201_OPEN_TEST == item_code) {
            ret = ft8201_open_test(&info, &tmp_result);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

        /* CB_TEST */
        if (CODE_FT8201_CB_TEST == item_code) {
            ret = ft8201_cb_test(&info, &tmp_result);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

        /* RAWDATA_TEST */
        if (CODE_FT8201_RAWDATA_TEST == item_code) {
            ret = ft8201_rawdata_test(&info, &tmp_result);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

        /* LCD_NOISE_TEST */
        if (CODE_FT8201_LCD_NOISE_TEST == item_code) {
            ret = ft8201_lcdnoise_test(&info, &tmp_result);
            if ((ret != 0) || (!tmp_result)) {
                test_result = false;
                test_data.test_item[item_count].testresult = RESULT_NG;
            } else
                test_data.test_item[item_count].testresult = RESULT_PASS;
        }

    }

    return test_result;
}

static void init_testitem_ft8201(char *ini)
{
    char str[MAX_KEYWORD_VALUE_LEN] = { 0 };

    FTS_TEST_FUNC_ENTER();
    /* RawData Test */
    GetPrivateProfileString("TestItem", "RAWDATA_TEST", "1", str, ini);
    ft8201_item.rawdata_test = fts_atoi(str);

    /* CB_TEST */
    GetPrivateProfileString("TestItem", "CB_TEST", "1", str, ini);
    ft8201_item.cb_test = fts_atoi(str);

    /* SHORT_CIRCUIT_TEST */
    GetPrivateProfileString("TestItem", "SHORT_CIRCUIT_TEST", "1", str, ini);
    ft8201_item.short_test = fts_atoi(str);

    /* LCD_NOISE_TEST */
    GetPrivateProfileString("TestItem", "LCD_NOISE_TEST", "0", str, ini);
    ft8201_item.lcd_noise_test = fts_atoi(str);

    /* OPEN_TEST */
    GetPrivateProfileString("TestItem", "OPEN_TEST", "0", str, ini);
    ft8201_item.open_test = fts_atoi(str);
    FTS_TEST_FUNC_EXIT();
}

static void init_basicthreshold_ft8201(char *ini)
{
    char str[MAX_KEYWORD_VALUE_LEN] = { 0 };

    FTS_TEST_FUNC_ENTER();
    /* RawData Test */
    GetPrivateProfileString("Basic_Threshold", "RawDataTest_Min", "5000", str, ini);
    ft8201_basic_thr.rawdata_test_min = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "RawDataTest_Max", "11000", str, ini);
    ft8201_basic_thr.rawdata_test_max = fts_atoi(str);

    /* CB Test */
    GetPrivateProfileString("Basic_Threshold", "CBTest_VA_Check", "1", str, ini);
    ft8201_basic_thr.cb_test_va_check = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "CBTest_Min", "3", str, ini);
    ft8201_basic_thr.cb_test_min = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "CBTest_Max", "100", str, ini);
    ft8201_basic_thr.cb_test_max = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "CBTest_VKey_Check", "1", str, ini);
    ft8201_basic_thr.cb_test_vk_check = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "CBTest_Min_Vkey", "3", str, ini);
    ft8201_basic_thr.cb_test_min_vk = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "CBTest_Max_Vkey", "100", str, ini);
    ft8201_basic_thr.cb_test_max_vk = fts_atoi(str);

    /* Short Circuit Test */
    GetPrivateProfileString("Basic_Threshold", "ShortCircuit_ResMin", "1000", str, ini);
    ft8201_basic_thr.short_res_min = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "ShortCircuit_VkResMin", "500", str, ini);
    ft8201_basic_thr.short_res_vk_min = fts_atoi(str);

    /* Lcd Noise Test */
    GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Frame", "50", str, ini);
    ft8201_basic_thr.lcd_noise_test_frame = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Max_Screen", "32", str, ini);
    ft8201_basic_thr.lcd_noise_test_max_screen = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Max_Frame", "32", str, ini);
    ft8201_basic_thr.lcd_noise_test_max_frame = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Coefficient", "50", str, ini);
    ft8201_basic_thr.lcd_noise_coefficient = fts_atoi(str);
    GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Coefficient_key", "50", str, ini);
    ft8201_basic_thr.lcd_noise_coefficient_key = fts_atoi(str);

    /* Open Test */
    GetPrivateProfileString("Basic_Threshold", "OpenTest_CBMin", "0", str, ini);
    ft8201_basic_thr.open_test_min = fts_atoi(str);
    FTS_TEST_FUNC_EXIT();
}

static void init_detailthreshold_ft8201(char *ini)
{
    FTS_TEST_FUNC_ENTER();
    OnInit_InvalidNode(ini);
    OnInit_DThreshold_RawDataTest(ini);
    OnInit_DThreshold_CBTest(ini);
    FTS_TEST_FUNC_EXIT();
}

static void set_testitem_sequence_ft8201(void)
{
    FTS_TEST_FUNC_ENTER();
    test_data.test_num = 0;
    /* Enter Factory Mode */
    fts_set_testitem(CODE_FT8201_ENTER_FACTORY_MODE);

    /* SHORT_CIRCUIT_TEST */
    if ( ft8201_item.short_test == 1) {
        fts_set_testitem(CODE_FT8201_SHORT_CIRCUIT_TEST) ;
    }

    /* OPEN_TEST */
    if ( ft8201_item.open_test == 1) {
        fts_set_testitem(CODE_FT8201_OPEN_TEST);
    }

    /* CB_TEST */
    if ( ft8201_item.cb_test == 1) {
        fts_set_testitem(CODE_FT8201_CB_TEST);
    }

    /* RawData Test */
    if ( ft8201_item.rawdata_test == 1) {
        fts_set_testitem(CODE_FT8201_RAWDATA_TEST);
    }

    /* LCD_NOISE_TEST */
    if ( ft8201_item.lcd_noise_test == 1) {
        fts_set_testitem(CODE_FT8201_LCD_NOISE_TEST);
    }
    FTS_TEST_FUNC_EXIT();
}

struct test_funcs test_func_ft8201 = {
    .ic_series = TEST_ICSERIES(IC_FT8201),
    .init_testitem = init_testitem_ft8201,
    .init_basicthreshold = init_basicthreshold_ft8201,
    .init_detailthreshold = init_detailthreshold_ft8201,
    .set_testitem_sequence  = set_testitem_sequence_ft8201,
    .start_test = start_test_ft8201,
};
