--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   21:20:51 06/03/2013
-- Design Name:   
-- Module Name:   D:/Dropbox/vWorker/freelancehandover/ise/tb_cmd_decoder.vhd
-- Project Name:  prehandover
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: cmd_decoder
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
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY tb_cmd_decoder IS
END tb_cmd_decoder;
 
ARCHITECTURE behavior OF tb_cmd_decoder IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT cmd_decoder
    PORT(
         status : OUT  std_logic_vector(15 downto 0);
         HDMI_out0_cmd : OUT  std_logic_vector(10 downto 0);
         HDMI_out1_cmd : OUT  std_logic_vector(10 downto 0);
         HDMI_out_source : OUT  std_logic;
         controller_cmd : OUT  std_logic_vector(3 downto 0);
         jpeg_encoder_cmd : OUT  std_logic_vector(2 downto 0);
         image_buffer_cmd : OUT  std_logic_vector(14 downto 0);
         HDMI_input0_cmd : OUT  std_logic;
         HDMI_input1_cmd : OUT  std_logic;
         toggle_hdmi_source : IN  std_logic;
         toggle_jpg_source : IN  std_logic;
         cmd_byte : IN  std_logic_vector(7 downto 0);
         cmd_en : IN  std_logic;
         rst_n : IN  std_logic;
         ifclk : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal toggle_hdmi_source : std_logic := '0';
   signal toggle_jpg_source : std_logic := '0';
   signal cmd_byte : std_logic_vector(7 downto 0) := (others => '0');
   signal cmd_en : std_logic := '0';
   signal rst_n : std_logic := '0';
   signal ifclk : std_logic := '0';

 	--Outputs
   signal status : std_logic_vector(15 downto 0);
   signal HDMI_out0_cmd : std_logic_vector(10 downto 0);
   signal HDMI_out1_cmd : std_logic_vector(10 downto 0);
   signal HDMI_out_source : std_logic;
   signal controller_cmd : std_logic_vector(3 downto 0);
   signal jpeg_encoder_cmd : std_logic_vector(2 downto 0);
   signal image_buffer_cmd : std_logic_vector(14 downto 0);
   signal HDMI_input0_cmd : std_logic;
   signal HDMI_input1_cmd : std_logic;

   -- Clock period definitions
   constant ifclk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: cmd_decoder PORT MAP (
          status => status,
          HDMI_out0_cmd => HDMI_out0_cmd,
          HDMI_out1_cmd => HDMI_out1_cmd,
          HDMI_out_source => HDMI_out_source,
          controller_cmd => controller_cmd,
          jpeg_encoder_cmd => jpeg_encoder_cmd,
          image_buffer_cmd => image_buffer_cmd,
          HDMI_input0_cmd => HDMI_input0_cmd,
          HDMI_input1_cmd => HDMI_input1_cmd,
          toggle_hdmi_source => toggle_hdmi_source,
          toggle_jpg_source => toggle_jpg_source,
          cmd_byte => cmd_byte,
          cmd_en => cmd_en,
          rst_n => rst_n,
          ifclk => ifclk
        );

   -- Clock process definitions
   ifclk_process :process
   begin
		ifclk <= '0';
		wait for ifclk_period/2;
		ifclk <= '1';
		wait for ifclk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	
	  rst_n <= '1';

      wait for ifclk_period*1000;

      -- insert stimulus here 
	  toggle_hdmi_source <= '1';

      wait;
   end process;

END;
