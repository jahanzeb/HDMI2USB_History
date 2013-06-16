--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   12:07:57 06/07/2013
-- Design Name:   
-- Module Name:   D:/Dropbox/vWorker/freelancehandover/ise/tb_raw_uvc.vhd
-- Project Name:  prehandover
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: raw_uvc
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
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;		 
USE ieee.std_logic_unsigned.all;
 
use IEEE.STD_LOGIC_TEXTIO.ALL;
  
library STD;
use STD.TEXTIO.ALL;  
 
ENTITY tb_raw_uvc IS
END tb_raw_uvc;
 
ARCHITECTURE behavior OF tb_raw_uvc IS 
 

  file f_capture           : text;
  constant CAPTURE_ORAM    : string := "..\sim\image_raw.txt";

component rgbRam is
port (
raddr : in std_logic_vector(19 downto 0);
clk : in std_logic;
q : out std_logic_vector(23 downto 0)
);
end component rgbRam;   

 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT raw_uvc
    PORT(
         raw_en : IN  std_logic;
         raw_bytes : IN  std_logic_vector(23 downto 0);
         to_send : IN  std_logic_vector(23 downto 0);
         raw_fifo_full : OUT  std_logic;
         error : OUT  std_logic;
         raw_clk : IN  std_logic;
         raw_enable : IN  std_logic;
         slwr : OUT  std_logic;
         pktend : OUT  std_logic;
         fdata : OUT  std_logic_vector(7 downto 0);
         flag_full : IN  std_logic;
         ifclk : IN  std_logic;
         faddr : IN  std_logic_vector(1 downto 0);
         uvcin : IN  std_logic_vector(1 downto 0);
         uvc_in_free : OUT  std_logic;
		 header 	: in std_logic;
		 sti 	: in std_logic;
         rst_n : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal raw_en : std_logic := '0';
   signal raw_bytes : std_logic_vector(23 downto 0) := (others => '0');
   signal to_send : std_logic_vector(23 downto 0) := (others => '0');
   signal to_send_bytes : std_logic_vector(23 downto 0) := (others => '0');
   signal raw_clk : std_logic := '0';
   signal raw_enable : std_logic := '0';
   signal flag_full : std_logic := '1';
   signal ifclk : std_logic := '0';
   signal faddr : std_logic_vector(1 downto 0) := (others => '0');
   signal uvcin : std_logic_vector(1 downto 0) := (others => '0');
   signal rst_n : std_logic := '1';

 	--Outputs
   signal raw_fifo_full : std_logic;
   signal error : std_logic;
   signal slwr : std_logic;
   signal pktend : std_logic;
   signal fdata : std_logic_vector(7 downto 0);
   signal uvc_in_free : std_logic;
   signal sim : std_logic:= '1';
   
   
   signal resx : std_logic_vector(15 downto 0) := (others => '0');
   signal resy : std_logic_vector(15 downto 0) := (others => '0');
   signal start : std_logic := '0';
   signal w_start2 : std_logic := '0';
   signal total_send : std_logic_vector(23 downto 0):=(others => '0');   


   -- Clock period definitions
   constant raw_clk_period : time := 10 ns;
   constant ifclk_period : time := 20 ns;
 
BEGIN
 
  p_capture : process
  variable fLine           : line;
  begin
    file_open(f_capture, CAPTURE_ORAM, write_mode);
   
    while sim = '1' loop--done /= '1' loop
      wait until rising_edge(ifclk); 
	  -- wait for 1 ns;
      
      if slwr = '0' then
        hwrite(fLine, fdata);
        writeline(f_capture, fLine);
      end if;
    
    end loop;
    
    file_close(f_capture);  
    wait;  
  end process;
 
	-- Instantiate the Unit Under Test (UUT)
   uut: raw_uvc PORT MAP (
          raw_en => raw_en,
          raw_bytes => raw_bytes,
          to_send => to_send_bytes,
          raw_fifo_full => raw_fifo_full,
          error => error,
          raw_clk => raw_clk,
          raw_enable => raw_enable,
          slwr => slwr,
          pktend => pktend,
          fdata => fdata,
          flag_full => flag_full,
          ifclk => ifclk,
          faddr => faddr,
          uvcin => uvcin,
          uvc_in_free => uvc_in_free,
			sti => '0',
		  header => '0',
          rst_n => rst_n
        );

   -- Clock process definitions
   raw_clk_process :process
   begin
		raw_clk <= '0';
		wait for raw_clk_period/2;
		raw_clk <= '1';
		wait for raw_clk_period/2;
   end process;
 
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
		rst_n  <= '0';
		wait for 100 ns;	
		rst_n  <= '1';
		uvcin <= "10";
		faddr <= "10";
		raw_enable <= '1';
		-- resx <= X"0500";resy <= X"02D0";to_send <= X"0E_10_00"; -- 1280×720  (921600)
		-- resx <= X"0400";resy <= X"0300";to_send <= X"0C_00_00"; -- 1024x768 (786432)
		-- resx <= X"0400";resy <= X"0080";to_send <= X"02_00_00"; -- 1024*128 -- 131072
		-- resx <= X"0400";resy <= X"0040";to_send <= X"01_00_00"; -- 1024*64 -- 65536	
		-- resx <= X"0400";resy <= X"0010";to_send <= X"00_40_00"; -- 1024*16 -- 16384	
		resx <= X"0400";resy <= X"0008";to_send <= X"00_20_00"; to_send_bytes <= X"00_60_00"; -- 1024*8 -- 8192			
		-- resx <= X"0008";resy <= X"0008";to_send <= X"00_00_40"; -- 8*8 -- 64	  

	
      wait for raw_clk_period*10;
	  start <= '1';

      -- insert stimulus here 

      wait;
   end process;
   
   
   
   data_proc:process(raw_clk)
	
   begin
   if rising_edge(raw_clk) then
   
   
	raw_en <= '0';	
	if raw_fifo_full = '0' and start = '1' and w_start2 = '0' then
	
		 if total_send = to_send then
			w_start2 <= '1';
			total_send <= (others => '0');
		else			
			total_send <= total_send + 1;
			raw_bytes <= total_send;			
			raw_en <= '1';	
		end if;
	end if;
	end if;
	
   end process;
   
   
-- rgbRamCom: rgbRam port map
-- (
-- raddr => total_send(19 downto 0), 
-- clk => raw_clk, 
-- q => raw_bytes
-- );   

END;
