

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_TEXTIO.ALL;

-- USE ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;


  
library STD;
use STD.TEXTIO.ALL;  
  
library unisim;
use unisim.vcomponents.all;


ENTITY tb_hdmi2usb IS
END tb_hdmi2usb;
 
ARCHITECTURE behavior OF tb_hdmi2usb IS 


	type char_file is file of character;

		
	file f_capture_jpg           : text;
	file f_capture_bin_jpg       : char_file;	
	
	constant CAPTURE_ORAM_jpg    : string := "..\sim\uvc.txt";
	constant CAPTURE_BIN_jpg     : string := "..\sim\uvc.jpg";
	

	file f_capture_cdc           : text;
	file f_capture_bin_cdc       : char_file;	
	constant CAPTURE_ORAM_cdc    : string := "..\sim\cdc.txt";
	constant CAPTURE_BIN_cdc     : string := "..\sim\cdc.jpg";
	
	
	constant cdcout : std_logic_vector(1 downto 0):= "00"; --ep 2
	constant cdcin  : std_logic_vector(1 downto 0):= "01"; --ep 4
	constant uvcin  : std_logic_vector(1 downto 0):= "10"; --ep 6	
	

	constant DEBUG_EN              		: integer	:= 0;   
	constant C3_MEMCLK_PERIOD 			: integer	:= 3200;
	constant C3_RST_ACT_LOW			 	: integer	:= 0;
	constant C3_INPUT_CLK_TYPE 			: string	:= "SINGLE_ENDED";
	constant C3_NUM_DQ_PINS        		: integer	:= 16;
	constant C3_MEM_ADDR_WIDTH     		: integer	:= 13;
	constant C3_MEM_BANKADDR_WIDTH 		: integer	:= 3;   
	constant C3_MEM_ADDR_ORDER     		: string	:= "ROW_BANK_COLUMN"; 
	constant C3_P0_MASK_SIZE 			: integer	:= 4;
	constant C3_P0_DATA_PORT_SIZE 		: integer	:= 32;  
	constant C3_P1_MASK_SIZE   			: integer	:= 4;
	constant C3_P1_DATA_PORT_SIZE  		: integer	:= 32;
	constant C3_MEM_BURST_LEN	 		: integer	:= 4;
	constant C3_MEM_NUM_COL_BITS   		: integer	:= 10;
	constant C3_SIMULATION      		: string	:= "TRUE";
	constant C3_CALIB_SOFT_IP      		: string	:= "TRUE";


	
	constant SIMULATION      		: string := "TRUE";	
 
 
	component ddr2_model_c3 is
	port (
		ck      : in    std_logic;
		ck_n    : in    std_logic;
		cke     : in    std_logic;
		cs_n    : in    std_logic;
		ras_n   : in    std_logic;
		cas_n   : in    std_logic;
		we_n    : in    std_logic;
		dm_rdqs : inout std_logic_vector(1 downto 0);
		ba      : in    std_logic_vector(2 downto 0);
		addr    : in    std_logic_vector(12 downto 0);
		dq      : inout std_logic_vector(15 downto 0);
		dqs     : inout std_logic_vector(1 downto 0);
		dqs_n   : inout std_logic_vector(1 downto 0);
		rdqs_n  : out   std_logic_vector(1 downto 0);
		odt     : in    std_logic
		);
	end component;	
	
	
	component hdmi2usb is 
	generic (
		SIMULATION             	  : string := "FALSE";
		C3_P0_MASK_SIZE           : integer := 4;
		C3_P0_DATA_PORT_SIZE      : integer := 32;
		C3_P1_MASK_SIZE           : integer := 4;
		C3_P1_DATA_PORT_SIZE      : integer := 32;
		C3_MEMCLK_PERIOD          : integer := 3200;
		C3_RST_ACT_LOW            : integer := 0;
		C3_INPUT_CLK_TYPE         : string := "SINGLE_ENDED";
		C3_CALIB_SOFT_IP          : string := "TRUE";
		C3_SIMULATION             : string := "FALSE";
		DEBUG_EN                  : integer := 0;
		C3_MEM_ADDR_ORDER         : string := "ROW_BANK_COLUMN";
		C3_NUM_DQ_PINS            : integer := 16;
		C3_MEM_ADDR_WIDTH         : integer := 13;
		C3_MEM_BANKADDR_WIDTH     : integer := 3
	);
	port
	(
		RX0_TMDS 	: in std_logic_vector(3 downto 0); 
		RX0_TMDSB 	: in std_logic_vector(3 downto 0);
		TX0_TMDS  	: out std_logic_vector(3 downto 0);
		TX0_TMDSB  	: out std_logic_vector(3 downto 0);

		RX1_TMDS  	: in std_logic_vector(3 downto 0);
		RX1_TMDSB  	: in std_logic_vector(3 downto 0);
		TX1_TMDS  	: out std_logic_vector(3 downto 0);
		TX1_TMDSB  	: out std_logic_vector(3 downto 0);

		scl_pc0 	: in std_logic; -- DDC scl connected with PC
		scl_lcd0 	: out std_logic; -- DDC scl connected with LCD
		sda_pc0 	: inout std_logic; -- DDC sda connected with PC
		sda_lcd0	: inout std_logic; -- DDC sda connected with LCD

		-- scl_pc1 	: in std_logic; -- DDC scl connected with PC
		scl_lcd1 	: out std_logic; -- DDC scl connected with LCD
		-- sda_pc1 	: inout std_logic; -- DDC sda connected with PC
		sda_lcd1 	: inout std_logic; -- DDC sda connected with LCD

		btnc 		: in std_logic;
		btnu		: in std_logic; 
		btnl		: in std_logic;
		btnr		: in std_logic; 
		btnd		: in std_logic; 

		LED 		: out std_logic_vector(7 downto 0);
		sw 			: in std_logic_vector(7 downto 0);

		-- USB Chip
		fdata 		: inout std_logic_vector(7 downto 0); 
		flagA 		: in std_logic;
		flagB 		: in std_logic; -- flag_full(flagB)
		flagC 		: in std_logic; -- flag_empty(flagC)
		faddr 		: out std_logic_vector(1 downto 0); 
		slwr 		: out std_logic;
		slrd 		: out std_logic;
		sloe 		: out std_logic;
		pktend 		: out std_logic;
		slcs 		: out std_logic; 
		ifclk 		: in std_logic; 


		-- DDR2 RAM
		mcb3_dram_dq 	: inout std_logic_vector(15 downto 0);
		mcb3_dram_a 	: out std_logic_vector(12 downto 0);
		mcb3_dram_ba 	: out std_logic_vector(2 downto 0);
		mcb3_dram_ras_n : out std_logic;
		mcb3_dram_cas_n	: out std_logic;
		mcb3_dram_we_n	: out std_logic;
		mcb3_dram_cke	: out std_logic;
		mcb3_dram_dm	: out std_logic;
		mcb3_dram_udqs 	: inout std_logic;
		mcb3_dram_udqs_n: inout std_logic;
		mcb3_rzq		: inout std_logic;
		mcb3_zio		: inout std_logic;
		mcb3_dram_udm	: out std_logic;
		mcb3_dram_odt	: out std_logic;
		mcb3_dram_dqs	: inout std_logic;
		mcb3_dram_dqs_n	: inout std_logic;
		mcb3_dram_ck	: out std_logic;
		mcb3_dram_ck_n	: out std_logic;

		rst_n : in std_logic;
		clk	: in std_logic
	);

	end component hdmi2usb;
	
	

	--BiDirs
   signal mcb3_dram_dq : std_logic_vector(15 downto 0);
   signal mcb3_dram_udqs : std_logic;
   signal mcb3_dram_udqs_n : std_logic;
   signal mcb3_rzq : std_logic;
   signal mcb3_zio : std_logic;
   signal mcb3_dram_dqs : std_logic;
   signal mcb3_dram_dqs_n : std_logic;

 	--Outputs
   signal mcb3_dram_a : std_logic_vector(12 downto 0);
   signal mcb3_dram_ba : std_logic_vector(2 downto 0);
   signal mcb3_dram_ras_n : std_logic;
   signal mcb3_dram_cas_n : std_logic;
   signal mcb3_dram_we_n : std_logic;
   signal mcb3_dram_cke : std_logic;
   signal mcb3_dram_dm : std_logic;
   signal mcb3_dram_udm : std_logic;
   signal mcb3_dram_odt : std_logic;
   signal mcb3_dram_ck : std_logic;
   signal mcb3_dram_ck_n : std_logic;
  
      
   signal mcb3_enable1 : std_logic;   
   signal mcb3_enable2 : std_logic;   
   signal mcb3_command : std_logic_vector(2 downto 0);
   signal mcb3_dram_dm_vector : std_logic_vector(1 downto 0);
   signal mcb3_dram_dqs_n_vector : std_logic_vector(1 downto 0);
   signal mcb3_dram_dqs_vector : std_logic_vector(1 downto 0);
   

   --Inputs
   signal clk : std_logic := '0';   
   signal rst_n : std_logic := '1';


   -- Clock period definitions
   constant clk_period : time := 10 ns;
   constant ifclk_period : time := 20.8333 ns; -- 48 MHz
   
   -- signals 
   signal RX0_TMDS : std_logic_vector(3 downto 0) := (others => '0');
   signal RX0_TMDSB : std_logic_vector(3 downto 0):= (others => '0');
   signal TX0_TMDS : std_logic_vector(3 downto 0):= (others => '0');
   signal TX0_TMDSB : std_logic_vector(3 downto 0):= (others => '0');
   signal RX1_TMDS : std_logic_vector(3 downto 0):= (others => '0');
   signal RX1_TMDSB : std_logic_vector(3 downto 0):= (others => '0');
   signal TX1_TMDS : std_logic_vector(3 downto 0):= (others => '0');
   signal sda_lcd0 : std_logic := '0';
   signal scl_lcd0 : std_logic := '0';
   signal sda_pc0 : std_logic := '0';
   signal scl_pc0 : std_logic := '0';
   signal TX1_TMDSB : std_logic_vector(3 downto 0):= (others => '0');
   signal btnc : std_logic := '0';
   signal scl_lcd1 : std_logic := '0';
   signal sda_lcd1 : std_logic := '0';
   signal btnl : std_logic := '0';
   signal btnu : std_logic := '0';
   signal btnr : std_logic := '0';
   signal sw : std_logic_vector(7 downto 0):= (others => '0');
   signal LED : std_logic_vector(7 downto 0):= (others => '0');
   signal fdata : std_logic_vector(7 downto 0):= (others => '0');
   signal fdata_i : std_logic_vector(7 downto 0):= (others => '0');
   signal btnd : std_logic := '0';
   signal flagA : std_logic := '0';
   signal flagB : std_logic := '1'; -- full 
   signal flagC : std_logic := '0'; -- empty
   signal slrd : std_logic := '1'; 
   signal slwr : std_logic := '1';
   signal faddr : std_logic_vector(1 downto 0):= (others => '0');
   signal pktend : std_logic := '1'; -- 
   signal slcs : std_logic := '0';
   signal sloe : std_logic := '1';
   signal ifclk : std_logic := '0';
   
   signal sim : std_logic := '1';

 
BEGIN
 
 
------------------------------------------------------------------------------------------------------
capture_uvc : process		
	variable fLine           : line;		
	begin		
		file_open(f_capture_jpg, CAPTURE_ORAM_jpg, write_mode);
		file_open(f_capture_bin_jpg, CAPTURE_BIN_jpg, write_mode);
		while sim = '1' loop
			wait until rising_edge(ifclk);
			if faddr = uvcin then
				if slwr = '0' then
					hwrite(fLine, fdata);
					writeline(f_capture_jpg, fLine);        
					write(f_capture_bin_jpg, CHARACTER'VAL(to_integer(unsigned(fdata))));        
				end if;
			end if;			
		end loop;
		file_close(f_capture_jpg);
		file_close(f_capture_bin_jpg);
		wait;  
end process capture_uvc; 

------------------------------------------------------------------------------------------------------------



capture_cdc : process		
	variable fLine           : line;		
	begin		
		file_open(f_capture_cdc, CAPTURE_ORAM_cdc, write_mode);
		-- file_open(f_capture_bin_cdc, CAPTURE_BIN_cdc, write_mode);
		while sim = '1' loop
			wait until rising_edge(ifclk);
			if faddr = cdcin then
				if slwr = '0' then
					hwrite(fLine, fdata);
					writeline(f_capture_cdc, fLine);        
					-- write(f_capture_bin_cdc, CHARACTER'VAL(to_integer(unsigned(fdata))));        
				end if;
			end if;			
		end loop;
		file_close(f_capture_cdc);
		-- file_close(f_capture_bin_cdc);
		wait;  
end process capture_cdc; 

 
-- Instantiate the Unit Under Test (UUT)
-- ========================================================================== --
-- Memory model instances                                                     -- 
-- ========================================================================== --
zio_pulldown3 : PULLDOWN port map(O => mcb3_zio);
rzq_pulldown3 : PULLDOWN port map(O => mcb3_rzq);

sdalcd1_pullup : PULLUP port map(O => sda_lcd1);
sdalcd0_pullup : PULLUP port map(O => sda_lcd0);


		
   
mcb3_command <= (mcb3_dram_ras_n & mcb3_dram_cas_n & mcb3_dram_we_n);

process(clk)
begin
  if (falling_edge(clk)) then
  -- if (rising_edge(clk)) then
	if (rst_n = '0') then
	  mcb3_enable1   <= '0';
	  mcb3_enable2 <= '0';
	elsif (mcb3_command = "100") then
	  mcb3_enable2 <= '0';
	elsif (mcb3_command = "101") then
	  mcb3_enable2 <= '1';
	else
	  mcb3_enable2 <= mcb3_enable2;
	end if;
	mcb3_enable1     <= mcb3_enable2;
  end if;
end process;

-----------------------------------------------------------------------------
--read
-----------------------------------------------------------------------------
mcb3_dram_dqs_vector(1 downto 0) <= (mcb3_dram_udqs & mcb3_dram_dqs) when (mcb3_enable2 = '0' and mcb3_enable1 = '0') else "ZZ";
mcb3_dram_dqs_n_vector(1 downto 0) <= (mcb3_dram_udqs_n & mcb3_dram_dqs_n) when (mcb3_enable2 = '0' and mcb3_enable1 = '0') else "ZZ";

-----------------------------------------------------------------------------
--write
-----------------------------------------------------------------------------
mcb3_dram_dqs <= mcb3_dram_dqs_vector(0) when ( mcb3_enable1 = '1') else 'Z';
mcb3_dram_udqs <= mcb3_dram_dqs_vector(1) when (mcb3_enable1 = '1') else 'Z';

mcb3_dram_dqs_n <= mcb3_dram_dqs_n_vector(0) when (mcb3_enable1 = '1') else 'Z';
mcb3_dram_udqs_n <= mcb3_dram_dqs_n_vector(1) when (mcb3_enable1 = '1') else 'Z';

mcb3_dram_dm_vector <= (mcb3_dram_udm & mcb3_dram_dm);

------------------------------------------------------------------------------
-- ram model
------------------------------------------------------------------------------

u_mem_c3 : ddr2_model_c3 port map(
	ck        => mcb3_dram_ck,
	ck_n      => mcb3_dram_ck_n,
	cke       => mcb3_dram_cke,
	cs_n      => '0',
	ras_n     => mcb3_dram_ras_n,
	cas_n     => mcb3_dram_cas_n,
	we_n      => mcb3_dram_we_n,
	dm_rdqs   => mcb3_dram_dm_vector ,
	ba        => mcb3_dram_ba,
	addr      => mcb3_dram_a,
	dq        => mcb3_dram_dq,
	dqs       => mcb3_dram_dqs_vector,
	dqs_n     => mcb3_dram_dqs_n_vector,
	rdqs_n    => open,
	odt       => mcb3_dram_odt
);
 
-- Instantiate the Unit Under Test (UUT)
uut : hdmi2usb
	generic map(SIMULATION            => SIMULATION,
		        C3_P0_MASK_SIZE       => C3_P0_MASK_SIZE,
		        C3_P0_DATA_PORT_SIZE  => C3_P0_DATA_PORT_SIZE,
		        C3_P1_MASK_SIZE       => C3_P1_MASK_SIZE,
		        C3_P1_DATA_PORT_SIZE  => C3_P1_DATA_PORT_SIZE,
		        C3_MEMCLK_PERIOD      => C3_MEMCLK_PERIOD,
		        C3_RST_ACT_LOW        => C3_RST_ACT_LOW,
		        C3_INPUT_CLK_TYPE     => C3_INPUT_CLK_TYPE,
		        C3_CALIB_SOFT_IP      => C3_CALIB_SOFT_IP,
		        C3_SIMULATION         => C3_SIMULATION,
		        DEBUG_EN              => DEBUG_EN,
		        C3_MEM_ADDR_ORDER     => C3_MEM_ADDR_ORDER,
		        C3_NUM_DQ_PINS        => C3_NUM_DQ_PINS,
		        C3_MEM_ADDR_WIDTH     => C3_MEM_ADDR_WIDTH,
		        C3_MEM_BANKADDR_WIDTH => C3_MEM_BANKADDR_WIDTH)
	port map(RX0_TMDS         => RX0_TMDS,
		     RX0_TMDSB        => RX0_TMDSB,
		     TX0_TMDS         => TX0_TMDS,
		     TX0_TMDSB        => TX0_TMDSB,
		     RX1_TMDS         => RX1_TMDS,
		     RX1_TMDSB        => RX1_TMDSB,
		     TX1_TMDS         => TX1_TMDS,
		     TX1_TMDSB        => TX1_TMDSB,
		     scl_pc0          => scl_pc0,
		     scl_lcd0         => scl_lcd0,
		     sda_pc0          => sda_pc0,
		     sda_lcd0         => sda_lcd0,
		     scl_lcd1         => scl_lcd1,
		     sda_lcd1         => sda_lcd1,
		     btnc             => btnc,
		     btnu             => btnu,
		     btnl             => btnl,
		     btnr             => btnr,
		     btnd             => btnd,
		     LED              => LED,
		     sw               => sw,
		     fdata            => fdata,
		     flagA            => flagA,
		     flagB            => flagB,
		     flagC            => flagC,
		     faddr            => faddr,
		     slwr             => slwr,
		     slrd             => slrd,
		     sloe             => sloe,
		     pktend           => pktend,
		     slcs             => slcs,
		     ifclk            => ifclk,
		     mcb3_dram_dq     => mcb3_dram_dq,
		     mcb3_dram_a      => mcb3_dram_a,
		     mcb3_dram_ba     => mcb3_dram_ba,
		     mcb3_dram_ras_n  => mcb3_dram_ras_n,
		     mcb3_dram_cas_n  => mcb3_dram_cas_n,
		     mcb3_dram_we_n   => mcb3_dram_we_n,
		     mcb3_dram_cke    => mcb3_dram_cke,
		     mcb3_dram_dm     => mcb3_dram_dm,
		     mcb3_dram_udqs   => mcb3_dram_udqs,
		     mcb3_dram_udqs_n => mcb3_dram_udqs_n,
		     mcb3_rzq         => mcb3_rzq,
		     mcb3_zio         => mcb3_zio,
		     mcb3_dram_udm    => mcb3_dram_udm,
		     mcb3_dram_odt    => mcb3_dram_odt,
		     mcb3_dram_dqs    => mcb3_dram_dqs,
		     mcb3_dram_dqs_n  => mcb3_dram_dqs_n,
		     mcb3_dram_ck     => mcb3_dram_ck,
		     mcb3_dram_ck_n   => mcb3_dram_ck_n,
		     rst_n            => rst_n,
		     clk              => clk);

-- Clock process definitions
clk_process :process
begin
	clk <= '0';
	wait for clk_period/2;
	clk <= '1';
	wait for clk_period/2;
end process;


ifclk_process :process
begin
	ifclk <= '0';
	wait for ifclk_period/2;
	ifclk <= '1';
	wait for ifclk_period/2;
end process;


fdata <= fdata_i when sloe = '0' else "ZZZZZZZZ";



-- Stimulus process
stim_proc: process
begin		

	rst_n <= '0';
	wait for 10 us;	
	rst_n <= '1';
	
	wait for 10 us; 

	-- flag_full(flagB)
	-- flag_empty(flagC)	
	
	-- Check status of USB Controller 
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"00";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"10";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"20";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"30";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"40";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"50";	
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	

	
	


	-- btnu <= '1';
	
	-- wait for 10 us;
	
	-- btnu <= '0';
	
	-- wait for 10 us;
	
	

	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"55";	-- Address
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);

	
	
	-- wait until faddr = cdcout;
	-- wait until rising_edge(ifclk);	
	-- flagC <= '1';	
	-- fdata_i <= X"4e";	-- CMD
	-- wait until slrd = '0';	
	-- wait until rising_edge(ifclk);
	-- flagC <= '0';
	-- wait until rising_edge(ifclk);
	
	-- wait until faddr = uvcin;
	-- wait until rising_edge(ifclk);	
	-- flagB <= '0';
	
	wait until pktend = '0';
	wait for 10*ifclk_period;
	
	wait until pktend = '0';
	wait for 10*ifclk_period;	

	wait until pktend = '0';
	wait for 100*ifclk_period;
	sim <= '0';	
	wait for 100*ifclk_period;	
	
	assert false report "end of simulation" severity failure;	
	
	wait;
end process;
 

END; -- Architecture 
