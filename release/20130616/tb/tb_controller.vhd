--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:36:19 05/28/2013
-- Design Name:   
-- Module Name:   D:/Dropbox/vWorker/freelancehandover/tb/tb_controller.vhd
-- Project Name:  prehandover
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: controller_top
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
library IEEE;
  use IEEE.STD_LOGIC_1164.all;
  use ieee.numeric_std.all;
  use IEEE.STD_LOGIC_TEXTIO.ALL;
  
library STD;
  use STD.TEXTIO.ALL;  
 
ENTITY tb_controller IS
END tb_controller;
 
ARCHITECTURE behavior OF tb_controller IS 
 
 
    type char_file is file of character;

  file f_capture           : text;
  file f_capture_bin       : char_file;
  constant CAPTURE_ORAM    : string := "..\sim\OUT_RAM.txt";
  constant CAPTURE_BIN     : string := "..\sim\test_out.jpg";
 
 
    -- Component Declaration for the Unit Under Test (UUT)
  
component jpeg_encoder_top_jpg
  port 
  (
        clk                : in  std_logic;
        rst_n              : in  std_logic;
        
        -- OUT RAM
        ram_byte           : out std_logic_vector(7 downto 0);
        ram_wren           : out std_logic;
        outif_almost_full  : in  std_logic;
		
		-- others
		jpeg_enable		   : in std_logic
		
   );
end component jpeg_encoder_top_jpg;


    COMPONENT controller_top
    PORT(
         edid0_byte : IN  std_logic_vector(7 downto 0);
         edid0_byte_en : IN  std_logic;
         edid1_byte : IN  std_logic_vector(7 downto 0);
         edid1_byte_en : IN  std_logic;
         jpeg_byte : IN  std_logic_vector(7 downto 0);
         jpeg_clk : IN  std_logic;
         jpeg_en : IN  std_logic;
         jpeg_enable : OUT  std_logic;
		 jpeg_fifo_full : out std_logic;	
         fdata : INOUT  std_logic_vector(7 downto 0);
         flag_full : IN  std_logic;
         flag_empty : IN  std_logic;
         faddr : OUT  std_logic_vector(1 downto 0);
         slwr : OUT  std_logic;
         slrd : OUT  std_logic;
         sloe : OUT  std_logic;
         pktend : OUT  std_logic;
         ifclk : IN  std_logic;
         resX0 : IN  std_logic_vector(15 downto 0);
         resY0 : IN  std_logic_vector(15 downto 0);
         resX1 : IN  std_logic_vector(15 downto 0);
         resY1 : IN  std_logic_vector(15 downto 0);
         jpeg_error : IN  std_logic;
         rgb_de0 : IN  std_logic;
         rgb_de1 : IN  std_logic;
         dvi_hdmi : OUT  std_logic_vector(3 downto 0);
         encoder_hdmi_source : OUT  std_logic_vector(1 downto 0);
         jpeg_quality : OUT  std_logic_vector(1 downto 0);
         encoder_output_type : OUT  std_logic_vector(1 downto 0);
         hdmi_out0_source : OUT  std_logic_vector(1 downto 0);
         hdmi_out1_source : OUT  std_logic_vector(1 downto 0);
         rst_n : IN  std_logic;
         clk : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal edid0_byte : std_logic_vector(7 downto 0) := (others => '0');
   signal edid0_byte_en : std_logic := '0';
   signal edid1_byte : std_logic_vector(7 downto 0) := (others => '0');
   signal edid1_byte_en : std_logic := '0';
   signal jpeg_byte : std_logic_vector(7 downto 0) := (others => '0');
   signal jpeg_clk : std_logic := '0';
   signal jpeg_en : std_logic := '0';
   signal flag_full : std_logic := '1';
   signal flag_empty : std_logic := '0';
   signal ifclk : std_logic := '0';
   signal resX0 : std_logic_vector(15 downto 0) := (others => '0');
   signal resY0 : std_logic_vector(15 downto 0) := (others => '0');
   signal resX1 : std_logic_vector(15 downto 0) := (others => '0');
   signal resY1 : std_logic_vector(15 downto 0) := (others => '0');
   signal jpeg_error : std_logic := '0';
   signal rgb_de0 : std_logic := '0';
   signal rgb_de1 : std_logic := '0';
   signal rst_n : std_logic := '0';
   signal clk : std_logic := '0';

	--BiDirs
   signal fdata : std_logic_vector(7 downto 0);

 	--Outputs
   signal sim : std_logic:= '1';
   signal jpeg_fifo_full : std_logic;
   signal jpeg_enable : std_logic;
   signal faddr : std_logic_vector(1 downto 0);
   signal slwr : std_logic;
   signal slrd : std_logic;
   signal sloe : std_logic;
   signal pktend : std_logic;
   signal dvi_hdmi : std_logic_vector(3 downto 0);
   signal encoder_hdmi_source : std_logic_vector(1 downto 0);
   signal jpeg_quality : std_logic_vector(1 downto 0);
   signal encoder_output_type : std_logic_vector(1 downto 0);
   signal hdmi_out0_source : std_logic_vector(1 downto 0);
   signal hdmi_out1_source : std_logic_vector(1 downto 0);

   -- Clock period definitions
   constant jpeg_clk_period : time := 10 ns;
   constant ifclk_period : time := 20.83 ns;
   constant clk_period : time := 10 ns;
 
BEGIN
 
 p_capture : process
    variable fLine           : line;
    variable fLine_bin       : line;
  begin
    file_open(f_capture, CAPTURE_ORAM, write_mode);
    file_open(f_capture_bin, CAPTURE_BIN, write_mode);
    
    while sim = '1' loop--done /= '1' loop
      wait until rising_edge(ifclk); 
      
      if slwr = '0' then
        hwrite(fLine, fdata);
        writeline(f_capture, fLine);
        
        write(f_capture_bin, CHARACTER'VAL(to_integer(unsigned(fdata))));
        
      end if;
    
    end loop;
    
    file_close(f_capture);
    file_close(f_capture_bin);
  
    wait;  
  end process; 
 
	-- Instantiate the Unit Under Test (UUT)
	
uut2: jpeg_encoder_top_jpg 
port map
(
clk    => clk,
rst_n         => rst_n,
ram_byte      => jpeg_byte,
ram_wren      => jpeg_en,
outif_almost_full =>jpeg_fifo_full,
jpeg_enable		 => jpeg_enable
);

	
	
   uut: controller_top PORT MAP (
          edid0_byte => edid0_byte,
          edid0_byte_en => edid0_byte_en,
          edid1_byte => edid1_byte,
          edid1_byte_en => edid1_byte_en,
          jpeg_byte => jpeg_byte,
          jpeg_clk => jpeg_clk,
          jpeg_en => jpeg_en,
          jpeg_enable => jpeg_enable,
		  jpeg_fifo_full => jpeg_fifo_full,
          fdata => fdata,
          flag_full => flag_full,
          flag_empty => flag_empty,
          faddr => faddr,
          slwr => slwr,
          slrd => slrd,
          sloe => sloe,
          pktend => pktend,
          ifclk => ifclk,
          resX0 => resX0,
          resY0 => resY0,
          resX1 => resX1,
          resY1 => resY1,
          jpeg_error => jpeg_error,
          rgb_de0 => rgb_de0,
          rgb_de1 => rgb_de1,
          dvi_hdmi => dvi_hdmi,
          encoder_hdmi_source => encoder_hdmi_source,
          jpeg_quality => jpeg_quality,
          encoder_output_type => encoder_output_type,
          hdmi_out0_source => hdmi_out0_source,
          hdmi_out1_source => hdmi_out1_source,
          rst_n => rst_n,
          clk => clk
        );

   -- Clock process definitions
   jpeg_clk_process :process
   begin
		jpeg_clk <= '0';
		wait for jpeg_clk_period/2;
		jpeg_clk <= '1';
		wait for jpeg_clk_period/2;
   end process;
 
   ifclk_process :process
   begin
		ifclk <= '0';
		wait for ifclk_period/2;
		ifclk <= '1';
		wait for ifclk_period/2;
   end process;
 
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
	  sim <= '1';
      wait for 100 ns;	
	  rst_n <= '1';
	  

      -- wait for jpeg_clk_period*10;
	  -- sim <= '1';



	  -- wait until faddr = "00"; 
	  
	  -- wait until rising_edge(ifclk);  
	  -- fdata <= X"08";
	  -- flag_empty <= '0';
	  
	  -- wait until rising_edge(ifclk);    
	  -- flag_empty <= '1';
	  -- fdata <= "ZZZZZZZZ";
	  

      -- insert stimulus here 

      wait;
   end process;

END;
