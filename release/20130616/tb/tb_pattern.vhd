--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   15:38:13 06/08/2013
-- Design Name:   
-- Module Name:   D:/Dropbox/vWorker/freelancehandover/tb/tb_pattern.vhd
-- Project Name:  prehandover
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: pattern
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
 
ENTITY tb_pattern IS
END tb_pattern;
 
ARCHITECTURE behavior OF tb_pattern IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT pattern
    PORT(
         red : OUT  std_logic_vector(7 downto 0);
         green : OUT  std_logic_vector(7 downto 0);
         blue : OUT  std_logic_vector(7 downto 0);
         resX : OUT  std_logic_vector(15 downto 0);
         resY : OUT  std_logic_vector(15 downto 0);
         de : OUT  std_logic;
         pclk : OUT  std_logic;
         vsync : OUT  std_logic;
         hsync : OUT  std_logic;
         clk : IN  std_logic;
         rst_n : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';
   signal rst_n : std_logic := '1';

 	--Outputs
   signal red : std_logic_vector(7 downto 0);
   signal green : std_logic_vector(7 downto 0);
   signal blue : std_logic_vector(7 downto 0);
   signal resX : std_logic_vector(15 downto 0);
   signal resY : std_logic_vector(15 downto 0);
   signal de : std_logic;
   signal pclk : std_logic;
   signal vsync : std_logic;
   signal hsync : std_logic;

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: pattern PORT MAP (
          red => red,
          green => green,
          blue => blue,
          resX => resX,
          resY => resY,
          de => de,
          pclk => pclk,
          vsync => vsync,
          hsync => hsync,
          clk => clk,
          rst_n => rst_n
        );

   -- Clock process definitions

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
	  rst_n <= '0';
      wait for 100 ns;	
	  rst_n <= '1';

      
      -- insert stimulus here 

      wait;
   end process;

END;
