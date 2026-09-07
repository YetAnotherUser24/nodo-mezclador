/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： snr8503x_nvr.c
 * 文件标识： 读校准值
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2021年11月12日
 *
 *******************************************************************************/
 // Read_Trim建议在初始化的过程中调用
 // Read_Trim会关闭全局中断
unsigned int Read_Trim(unsigned int addr);              //有效地址范围((addr >= 0x00000190) && (addr <= 0x0000001FF)) || ((addr >= 0x0000024C) && (addr <= 0x000000258))
void Prog_Trim(unsigned int addr, unsigned int data);   //有效地址范围((addr >= 0x000001E0) && (addr <= 0x0000001FF))
/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR *****END OF FILE****/
