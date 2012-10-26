/*
 * Xylon logiCVC frame buffer driver - pixel clock generation using clkgen IP core
 *
 * Author: Xylon d.o.o.
 * e-mail: gpantar@logicbricks.com
 *
 * 2012 (c) Xylon d.o.o.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


/***************************** Include Files *********************************/



#define NUM_CLK_OUTPUTS 6
#define NUM_CLK_REGS 21

#define	CKLGEN_RST_REG_OFF		(0)
#define	CKLGEN_PLL_OFF			(1)
#define	CKLGEN_RAM_OFF			(3)
#define	CLKGEN_PLL_RST_MASK		0x1

struct clkgen_freq_out
{
    long freq_out_hz[NUM_CLK_OUTPUTS];
};


/** Calculates the output registers depending on the freq_out and  c_osc_clk_freq_hz inputs,
 *  and writes them to array of NUM_CLK_REGS over regs_out pointer*/
void clkgen_calc_regs(struct clkgen_freq_out *freq_out, long c_clk_freq_init_hz, long c_osc_clk_freq_hz, long *regs_out);
