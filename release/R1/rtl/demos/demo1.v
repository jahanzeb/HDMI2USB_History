//////////////////////////////////////////////////////////////////////////////
/// Copyright (c) 2012, Jahanzeb Ahmad
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification, 
/// are permitted provided that the following conditions are met:
///
///  * Redistributions of source code must retain the above copyright notice, 
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice, 
///    this list of conditions and the following disclaimer in the documentation and/or 
///    other materials provided with the distribution.
///
///    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
///    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
///    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
///    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
///    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
///    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
///    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
///    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
///    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
///    POSSIBILITY OF SUCH DAMAGE.
///  * http://opensource.org/licenses/MIT
///  * http://copyfree.org/licenses/mit/license.txt
///
//////////////////////////////////////////////////////////////////////////////
/*!
 Demo 1: Test the HDMI and EDID modules and USB communication 
 
 Usefull links are
 
 http://www.xilinx.com/support/documentation/application_notes/xapp495_S6TMDS_Video_Interface.pdf
 
 http://www.xilinx.com/support/documentation/spartan-6.htm
 
 http://www.evernew.com.tw/HDMISpecification13a.pdf
 
 http://read.pudn.com/downloads110/ebook/456020/E-EDID%20Standard.pdf
 
 http://www.nxp.com/documents/user_manual/UM10204.pdf
 
 http://www.digilentinc.com/Products/Detail.cfm?NavPath=2,400,836&Prod=ATLYS
 
 http://en.wikipedia.org/wiki/Extended_display_identification_data
 
 http://en.wikipedia.org/wiki/I%C2%B2C
 
 http://en.wikipedia.org/wiki/Hdmi
 
*/

module demo1(
input wire rst_n,    //% The pink reset button active low
input wire clk,      //% 100 MHz osicallator
input wire [3:0]  RX0_TMDS, //% HDMI RX
input wire [3:0]  RX0_TMDSB, //% HDMI RX
output wire [3:0] TX0_TMDS, //% HDMI TX
output wire [3:0] TX0_TMDSB, //% HDMI TX
input  wire [2:0] SW,
output wire [7:0] LED,
input wire scl_pc, //% DDC scl connected with PC
output wire scl_lcd, //% DDC scl connected with LCD
inout wire sda_pc, //% DDC sda connected with PC
inout wire sda_lcd, //% DDC sda connected with LCD

//-- USB Chip
inout wire [7:0] fdata, //% USB chip data port
input wire flagA,flagB,flagC, //% USB chip falgs
output wire [1:0] faddr, //% USB fifo select
output wire slwr,slrd,sloe,pktend,slcs, //% USB fifo signals 
input wire ifclk //% Clock for USB fifo
);

//% internal wires and regs
// wire clk;

wire [7:0] sda_byte;
wire clk_100;

//% combinational logic
assign LED[0] = 0;
assign LED[1] = 0;
assign LED[2] = 0;
assign LED[3] = vsync;
assign LED[4] = rgb_de;
assign LED[5] = (flagB);
assign LED[6] = (flagC);

assign rst = ~ rst_n;
assign slcs = 0;
assign jpeg_enable = SW[2];
assign clk_100 = clk;

// -- FLAGA=PF, FLAGB=FF, FLAGC=EF, FLAGD=EP2PF 
//% usb process
//% main clk for this process is ifclk
usb_mjpeg usbComp(
.clk(clk_100),
.rst_n(rst_n),
.sda_byte(sda_byte),
.sda_en(sda_byte_en),
.jpeg_byte(ram_byte),
.jpeg_clk(clk_jpeg),
.jpeg_en(ram_wren),
.fdata(fdata),
.flag_full(flagB),
.flag_empty(flagC),
.faddr(faddr),
.slwr(slwr),
.slrd(slrd),
.sloe(sloe),
.pktend(pktend),
.ifclk(ifclk),
.jpeg_enable(jpeg_enable)
);


//% EDID hack unit, master slave based design
edid_master_slave_hack edid_hack(
.rst_n(rst_n),
.clk(clk_100),
.sda_lcd(sda_lcd),
.scl_lcd(scl_lcd),
.sda_pc(sda_pc),
.scl_pc(scl_pc),
.hpd_lcd(SW[0]),
.hpd_pc(LED[7]),
.sda_byte(sda_byte),
.sda_byte_en(sda_byte_en)
);

//% HDMI decoder and encoder  
dvi_demo hdmi_RX_TX(
.rst_n(rst_n),    
.clk(clk_100),       
.RX0_TMDS(RX0_TMDS),
.RX0_TMDSB(RX0_TMDSB),
.TX0_TMDS(TX0_TMDS),
.TX0_TMDSB(TX0_TMDSB),  
.rgb(rgb), //% raw RGB 
.rgb_de(rgb_de), //%  RGB en
.hsync(hsync),
.vsync(vsync),
.pclk(pclk),
.SW(SW[1]),	
.clk10x(clk10x)
);



endmodule
