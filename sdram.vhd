library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sdram is
    port(
        CLOCK_50     : IN  STD_LOGIC;
        LEDR         : OUT STD_LOGIC_VECTOR(9 downto 0);  -- LEDs for debugging
        KEY          : IN  STD_LOGIC_VECTOR(3 downto 0);

        -- SDRAM Interface
        DRAM_ADDR    : OUT STD_LOGIC_VECTOR(12 downto 0);
        DRAM_BA      : OUT STD_LOGIC_VECTOR(1 downto 0);
        DRAM_CAS_N   : OUT STD_LOGIC;
        DRAM_CKE     : OUT STD_LOGIC;
        DRAM_CLK     : OUT STD_LOGIC;  -- DRAM clock directly assigned from RAMSYS
        DRAM_CS_N    : OUT STD_LOGIC;
        DRAM_DQ      : INOUT STD_LOGIC_VECTOR(15 downto 0);
        DRAM_RAS_N   : OUT STD_LOGIC;
        DRAM_WE_N    : OUT STD_LOGIC;
        DRAM_LDQM, DRAM_UDQM : OUT STD_LOGIC;

        -- UART Interface
        UART_RX      : IN  STD_LOGIC;
        UART_TX      : OUT STD_LOGIC
    );
end sdram;

architecture main of sdram is
    -- SDRAM Control Signals
    signal SDRAM_ADDR       : STD_LOGIC_VECTOR(21 downto 0) := (others => '0');
    signal SDRAM_WE_N, SDRAM_RE_N : STD_LOGIC := '1';
    signal SDRAM_WRITEDATA  : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
    signal SDRAM_READDATA   : STD_LOGIC_VECTOR(15 downto 0);
    signal SDRAM_CS         : std_logic := '0';
    signal SDRAM_BE_N       : STD_LOGIC_VECTOR(1 downto 0) := "00"; -- Default byte enable for SDRAM

    -- UART Data Signals
    signal uart_tx_data     : std_logic_vector(7 downto 0);
    signal uart_tx_start_sig, uart_tx_done : std_logic := '0'; -- Internal UART control signal
    signal sdram_data_ready_sig : std_logic := '0'; -- Internal SDRAM data ready signal

    -- Data transfer signal
    signal data_to_uart : std_logic_vector(15 downto 0) := (others => '0'); -- Declaring data_to_uart

    -- State Machine Signals
    type state_type is (IDLE, WRITE_INIT, WAIT_WRITE, READ, WAIT_READ, SEND_LOW_BYTE, SEND_HIGH_BYTE, DONE);
    signal state : state_type := IDLE;
    signal address_counter : integer := 0;

    signal clk167, clk167_2, clk49_5: std_logic;

    -- SDRAM Debug Signals
    signal DRAM_DQM : STD_LOGIC_VECTOR(1 downto 0) := (others => '0');

    -- Dual-Port RAM signals
    signal ram_data_a, ram_data_b : std_logic_vector(7 downto 0);
    signal ram_q_a, ram_q_b : std_logic_vector(7 downto 0);
    signal ram_addr_a, ram_addr_b : integer range 0 to 511 := 0;
    signal ram_we_a, ram_we_b : std_logic := '0';

    -- SDRAM Controller component (RAMSYS) declaration
    component ramsys is
        port (
            clk_clk             : in std_logic;
            reset_reset_n       : in std_logic;
            clk167_shift_clk    : out std_logic;
            clk167_clk          : out std_logic;          -- Generates the 167 MHz clock
            clk49_5_clk         : out std_logic;
            wire_addr           : out std_logic_vector(12 downto 0);
            wire_ba             : out std_logic_vector(1 downto 0);
            wire_cas_n          : out std_logic;
            wire_cke            : out std_logic;
            wire_cs_n           : out std_logic;
            wire_dq             : inout std_logic_vector(15 downto 0);
            wire_dqm            : out std_logic_vector(1 downto 0);
            wire_ras_n          : out std_logic;
            wire_we_n           : out std_logic;
            sdram_address       : in std_logic_vector(21 downto 0);
            sdram_byteenable_n  : in std_logic_vector(1 downto 0);
            sdram_chipselect    : in std_logic;
            sdram_writedata     : in std_logic_vector(15 downto 0);
            sdram_read_n        : in std_logic;
            sdram_write_n       : in std_logic;
            sdram_readdata      : out std_logic_vector(15 downto 0);
            sdram_readdatavalid : out std_logic;
            sdram_waitrequest   : out std_logic
        );
    end component;

    -- UART component declaration
    component uart is
        port (
            clk          : in std_logic;
            rst_n        : in std_logic;
            rx           : in std_logic;
            tx           : out std_logic;
            rd_uart      : in std_logic;
            wr_uart      : in std_logic;
            wr_data      : in std_logic_vector(7 downto 0);
            rd_data      : out std_logic_vector(7 downto 0);
            rx_empty     : out std_logic;
            tx_full      : out std_logic
        );
    end component;

    -- Dual-Port RAM component declaration
    component true_dual_port_ram_dual_clock is
        port(
            clk_a   : in std_logic;
            clk_b   : in std_logic;
            addr_a  : in natural range 0 to 511;
            addr_b  : in natural range 0 to 511;
            data_a  : in std_logic_vector(7 downto 0);
            data_b  : in std_logic_vector(7 downto 0);
            we_a    : in std_logic;
            we_b    : in std_logic;
            q_a     : out std_logic_vector(7 downto 0);
            q_b     : out std_logic_vector(7 downto 0)
        );
    end component;

begin
    -- SDRAM Controller instance
    u0 : ramsys
        port map (
            clk_clk             => CLOCK_50,
            reset_reset_n       => KEY(0),
            clk167_shift_clk    => clk167_2,
            clk167_clk          => clk167,     -- Connects internally generated 167 MHz clock to DRAM_CLK
            clk49_5_clk         => clk49_5,
            wire_addr           => DRAM_ADDR,      -- SDRAM address lines
            wire_ba             => DRAM_BA,        -- SDRAM bank address
            wire_cas_n          => DRAM_CAS_N,     -- Column address select
            wire_cke            => DRAM_CKE,       -- Clock enable for SDRAM
            wire_cs_n           => DRAM_CS_N,      -- Chip select for SDRAM
            wire_dq             => DRAM_DQ,        -- Data lines for SDRAM
            wire_dqm            => DRAM_DQM,       -- Data mask for SDRAM
            wire_ras_n          => DRAM_RAS_N,     -- Row address select
            wire_we_n           => DRAM_WE_N,     -- Write enable
            sdram_address       => SDRAM_ADDR,     -- Internal SDRAM address
            sdram_byteenable_n  => SDRAM_BE_N,     -- Byte enable
            sdram_chipselect    => SDRAM_CS,       -- Chip select signal
            sdram_writedata     => SDRAM_WRITEDATA,-- Data to write to SDRAM
            sdram_read_n        => SDRAM_RE_N,     -- Read enable
            sdram_write_n       => SDRAM_WE_N,     -- Write enable
            sdram_readdata      => SDRAM_READDATA, -- Data read from SDRAM
            sdram_readdatavalid => open,           -- Valid read data signal (optional)
            sdram_waitrequest   => open            -- Wait request (optional)
        );

    -- UART Instance
    uart_inst : uart
        port map (
            clk          => CLOCK_50,
            rst_n        => KEY(0),
            rx           => UART_RX,
            tx           => UART_TX,
            rd_uart      => '0',
            wr_uart      => uart_tx_start_sig, 
            wr_data      => uart_tx_data,
            rd_data      => open,
            rx_empty     => open,
            tx_full      => open
        );

    -- Dual-Port RAM Instance
    ram_inst : true_dual_port_ram_dual_clock
        port map (
            clk_a  => clk167,      -- SDRAM clock domain
            clk_b  => CLOCK_50,    -- UART clock domain
            addr_a => ram_addr_a,  -- SDRAM side address
            addr_b => ram_addr_b,  -- UART side address
            data_a => ram_data_a,  -- Data to write from SDRAM
            data_b => uart_tx_data, -- Data to write from UART side
            we_a   => ram_we_a,    -- Write enable for SDRAM side
            we_b   => '0',         -- UART side is read-only in this configuration
            q_a    => ram_q_a,     -- Data output to SDRAM side
            q_b    => ram_q_b      -- Data output to UART side
        );

   -- Main FSM Process
  process (clk167, CLOCK_50)
  begin
    if rising_edge(clk167) then
      if KEY(0) = '0' then
        -- Reset all signals
        SDRAM_ADDR <= (others => '0');
        SDRAM_WE_N <= '1';
        SDRAM_RE_N <= '1';
        SDRAM_WRITEDATA <= (others => '0');
        state <= IDLE;
        address_counter <= 0;
        sdram_data_ready_sig <= '0';
      else
        case state is
          when IDLE =>
            SDRAM_WRITEDATA <= x"AAAA"; -- Data to write
            SDRAM_ADDR <= std_logic_vector(to_unsigned(address_counter, 22));
            state <= WRITE_INIT;

          when WRITE_INIT =>
            SDRAM_WE_N <= '0'; -- Start write operation
            state <= WAIT_WRITE;

          when WAIT_WRITE =>
            SDRAM_WE_N <= '1'; -- Complete write pulse
            if address_counter < 9 then
              address_counter <= address_counter + 1;
              SDRAM_ADDR <= std_logic_vector(to_unsigned(address_counter, 22));
              state <= WRITE_INIT; -- Loop back for next write
            else
              address_counter <= 0; -- Reset for read phase
              state <= READ;
            end if;

          when READ =>
            SDRAM_RE_N <= '0'; -- Start read operation
            SDRAM_ADDR <= std_logic_vector(to_unsigned(address_counter, 22));
            state <= WAIT_READ;

          when WAIT_READ =>
            SDRAM_RE_N <= '1'; -- Complete read pulse
            ram_data_a <= SDRAM_READDATA(7 downto 0); -- Write lower byte to RAM
            ram_addr_a <= address_counter;
            ram_we_a <= '1';
            data_to_uart <= SDRAM_READDATA; -- Capture data read from SDRAM
            sdram_data_ready_sig <= '1'; -- Signal UART to send data
            state <= SEND_LOW_BYTE;

          when SEND_LOW_BYTE =>
            uart_tx_data <= data_to_uart(7 downto 0); -- Send lower byte
            uart_tx_start_sig <= '1';
            state <= SEND_HIGH_BYTE;

          when SEND_HIGH_BYTE =>
            if uart_tx_done = '1' then
              uart_tx_data <= data_to_uart(15 downto 8); -- Send upper byte
              uart_tx_start_sig <= '1';
              sdram_data_ready_sig <= '0'; -- Clear data ready for next SDRAM read
              if address_counter < 9 then
                address_counter <= address_counter + 1;
                state <= READ; -- Loop back to read next address
              else
                state <= DONE; -- All addresses read, move to DONE
              end if;
            end if;

          when DONE =>
            -- Reset FSM to IDLE after completion or await further instruction
            state <= IDLE;

          when others =>
            state <= IDLE;
        end case;
      end if;
    end if;
  end process;

  -- UART Transmission Process (50 MHz)
  process (CLOCK_50)
  begin
    if rising_edge(CLOCK_50) then
      if sdram_data_ready_sig = '1' then
        uart_tx_start_sig <= '1'; -- Trigger UART transmission
        sdram_data_ready_sig <= '0'; -- Clear the flag to avoid multiple triggers
      else
        uart_tx_start_sig <= '0'; -- Stop UART transmission after completion
      end if;
    end if;
  end process;

  -- Assign internal signals to output signals
  --uart_tx_start <= uart_tx_start_sig;

end architecture main;





-- Original SDRAM VGA Code Below




--library ieee;
--
--use ieee.std_logic_1164.all;
--use ieee.numeric_std.all;
--use work.pcg.all;
--
--
--entity sdram is
--
--port(
--
--CLOCK_50: IN STD_LOGIC;
--SW: IN STD_LOGIC_VECTOR(9 downto 0);
------------------VGA-Interface---------------------
--VGA_B,VGA_G,VGA_R : OUT STD_LOGIC_VECTOR(7 downto 0);
--VGA_CLK,VGA_BLANK_N,VGA_HS,VGA_VS,VGA_SYNC_N: OUT STD_LOGIC;
--LEDR: OUT STD_LOGIC_VECTOR(9 downto 0);
--KEY: IN STD_LOGIC_VECTOR(3 downto 0);
--------------------SDRAM---------------------------
--DRAM_ADDR: OUT STD_LOGIC_VECTOR(12 downto 0);
--DRAM_BA: OUT STD_LOGIC_VECTOR(1 downto 0);
--DRAM_CAS_N: OUT STD_LOGIC;
--DRAM_CKE: OUT STD_LOGIC;
--DRAM_CLK: OUT STD_LOGIC;
--DRAM_CS_N: OUT STD_LOGIC;
--DRAM_DQ: INOUT STD_LOGIC_VECTOR(15 downto 0);
--DRAM_RAS_N: OUT STD_LOGIC;
--DRAM_WE_N: OUT STD_LOGIC;
--DRAM_LDQM,DRAM_UDQM: OUT STD_LOGIC
--);
--
--
--end sdram;
--
--
--architecture  main of sdram is
--TYPE STAGES IS (ST0,ST1);
--SIGNAL BUFF_CTRL: STAGES:=ST0;
-----------------------------test signals----------------------------
--signal counter : integer range 0 to 1000;
--signal test: std_logic:='0';
--signal testdata: std_logic_vector(7 downto 0):="00000000";
--signal Xpos,Ypos: integer range 0 to 799:=0;
-----------------------------sync-----------------------------
--signal BUFF_WAIT: std_logic:='0';
--signal VGAFLAG: std_logic_vector(2 downto 0);
---------------------------ram/gray----------------------------
--signal RAMFULL_POINTER:integer range 0 to 511:=0;
--signal RAMRESTART_POINTER: integer range 0 to 511:=0;
--signal RAMADDR1GR,RAMADDR2GR: std_logic_vector(8 downto 0):=(others=>'0');
--signal RAMADDR1GR_sync0,RAMADDR1GR_sync1,RAMADDR1GR_sync2,RAMADDR1_bin: std_logic_vector(8 downto 0);
--signal RAMADDR2GR_sync0,RAMADDR2GR_sync1,RAMADDR2GR_sync2,RAMADDR2_bin: std_logic_vector(8 downto 0);
---------------------------dual ram ----------------------------------
--signal RAMIN1,RAMIN2,RAMOUT1,RAMOUT2: std_logic_vector(7 downto 0);
--signal RAMWE1,RAMWE2: std_logic:='0';
--signal RAMADDR1,RAMADDR2: integer range 0 to 511:=0;
--------------------vga----------------------------------
--signal NEXTFRAME: std_logic_vector(2 downto 0):="000";
--signal FRAMEEND,FRAMESTART: std_logic:='0';
--signal ACTVIDEO: std_logic:='0';
--signal VGABEGIN: std_logic:='0';
--signal RED,GREEN,BLUE: STD_LOGIC_VECTOR(7 downto 0);
--------------------clock--------------------------------
--SIGNAL CLK143,CLK143_2,CLK49_5: STD_LOGIC;
--------------------sdram--------------------------------
--SIGNAL SDRAM_ADDR: STD_LOGIC_VECTOR(24 downto 0);
--SIGNAL SDRAM_BE_N: STD_LOGIC_VECTOR(1 downto 0);
--SIGNAL SDRAM_CS: STD_LOGIC;
--SIGNAL SDRAM_RDVAL,SDRAM_WAIT:STD_LOGIC;
--SIGNAL SDRAM_RE_N,SDRAM_WE_N: STD_LOGIC;
--SIGNAL SDRAM_READDATA,SDRAM_WRITEDATA: STD_LOGIC_VECTOR(15 downto 0);
--SIGNAL DRAM_DQM : STD_LOGIC_VECTOR(1 downto 0);
--
----------------------------------------------------------
--
--	component true_dual_port_ram_dual_clock is
--	port 
--	(
--		clk_a	: in std_logic;
--		clk_b	: in std_logic;
--		addr_a	: in natural range 0 to 511;
--		addr_b	: in natural range 0 to 511;
--		data_a	: in std_logic_vector(7 downto 0);
--		data_b	: in std_logic_vector(7 downto 0);
--		we_a	: in std_logic := '1';
--		we_b	: in std_logic := '1';
--		q_a		: out std_logic_vector(7 downto 0);
--		q_b		: out std_logic_vector(7 downto 0)
--	);
--	end component true_dual_port_ram_dual_clock;
--   component  vga is
--	port(
--		CLK: in std_logic;
--		R_OUT,G_OUT,B_OUT: OUT std_logic_vector(7 downto 0);
--		R_IN,G_IN,B_IN: IN std_logic_vector(7 downto 0);
--		VGAHS, VGAVS:OUT std_logic;
--	   ACTVID: OUT std_logic;
--		VGA_FRAMESTART: out std_logic;
--		VGA_FRAMEEND: out std_logic
--	);
--end component vga;
--
--
--component ramsys is
--        port (
--            clk_clk             : in    std_logic                     := 'X';             -- clk
--            reset_reset_n       : in    std_logic                     := 'X';             -- reset_n
--            clk143_shift_clk    : out   std_logic;                                        -- clk
--            clk143_clk          : out   std_logic;                                        -- clk
--            clk49_5_clk         : out   std_logic;                                        -- clk
--            wire_addr           : out   std_logic_vector(12 downto 0);                    -- addr
--            wire_ba             : out   std_logic_vector(1 downto 0);                     -- ba
--            wire_cas_n          : out   std_logic;                                        -- cas_n
--            wire_cke            : out   std_logic;                                        -- cke
--            wire_cs_n           : out   std_logic;                                        -- cs_n
--            wire_dq             : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
--            wire_dqm            : out   std_logic_vector(1 downto 0);                     -- dqm
--            wire_ras_n          : out   std_logic;                                        -- ras_n
--            wire_we_n           : out   std_logic;                                        -- we_n
--            sdram_address       : in    std_logic_vector(24 downto 0) := (others => 'X'); -- address
--            sdram_byteenable_n  : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- byteenable_n
--            sdram_chipselect    : in    std_logic                     := 'X';             -- chipselect
--            sdram_writedata     : in    std_logic_vector(15 downto 0) := (others => 'X'); -- writedata
--            sdram_read_n        : in    std_logic                     := 'X';             -- read_n
--            sdram_write_n       : in    std_logic                     := 'X';             -- write_n
--            sdram_readdata      : out   std_logic_vector(15 downto 0);                    -- readdata
--            sdram_readdatavalid : out   std_logic;                                        -- readdatavalid
--            sdram_waitrequest   : out   std_logic                                         -- waitrequest
--        );
--    end component ramsys;
--begin
--
--
--	u0 : component ramsys
--        port map (
--            clk_clk             => CLOCK_50,             --          clk.clk
--            reset_reset_n       => '1',       --        reset.reset_n
--            clk143_shift_clk    => CLK143_2,    -- clk143_shift.clk
--            clk143_clk          => CLK143,          --       clk143.clk
--            clk49_5_clk         => CLK49_5,         --      clk49_5.clk
--            wire_addr           => DRAM_ADDR,           --         wire.addr
--            wire_ba             => DRAM_BA,             --             .ba
--            wire_cas_n          => DRAM_CAS_N,          --             .cas_n
--            wire_cke            => DRAM_CKE,            --             .cke
--            wire_cs_n           => DRAM_CS_N,           --             .cs_n
--            wire_dq             => DRAM_DQ,             --             .dq
--            wire_dqm            => DRAM_DQM,            --             .dqm
--            wire_ras_n          => DRAM_RAS_N,          --             .ras_n
--            wire_we_n           => DRAM_WE_N,           --             .we_n
--            sdram_address       => SDRAM_ADDR,       --        sdram.address
--            sdram_byteenable_n  => SDRAM_BE_N,  --             .byteenable_n
--            sdram_chipselect    => SDRAM_CS,    --             .chipselect
--            sdram_writedata     => SDRAM_WRITEDATA,     --             .writedata
--            sdram_read_n        => SDRAM_RE_N,        --             .read_n
--            sdram_write_n       => SDRAM_WE_N,       --             .write_n
--            sdram_readdata      => SDRAM_READDATA,      --             .readdata
--            sdram_readdatavalid => SDRAM_RDVAL, --             .readdatavalid
--            sdram_waitrequest   => SDRAM_WAIT    --             .waitrequest
--        );
--
--      u1 : component vga 
--			port map(
--					CLK=>CLK49_5,
--					R_OUT=>VGA_R,
--					G_OUT=>VGA_G,
--					B_OUT=>VGA_B,
--					R_IN=>RED,
--					G_IN=>GREEN,
--					B_IN=>BLUE,
--					VGAHS=>VGA_HS,
--					VGAVS=>VGA_VS,
--				   ACTVID=>ACTVIDEO,
--					VGA_FRAMESTART=>FRAMESTART,
--					VGA_FRAMEEND=>FRAMEEND
--			);
--
--		u3: component true_dual_port_ram_dual_clock
--		   port map  (
--			clk_a=>CLK143,
--			clk_b=>clk49_5,
--			addr_a=>RAMADDR1,
--			addr_b=>RAMADDR2,
--			data_a=>RAMIN1,
--			data_b=>RAMIN2,
--			we_a=>RAMWE1,
--			we_b=>RAMWE2,
--			q_a=>RAMOUT1,
--			q_b=>RAMOUT2			
--			);
--
--
--
--
--
--DRAM_LDQM<=DRAM_DQM(0);
--DRAM_UDQM<=DRAM_DQM(1);
--DRAM_CLK<=CLK143_2;
--VGA_CLK<=CLK49_5;
--SDRAM_CS<='1';
--SDRAM_BE_N<="00";
--VGA_BLANK_N<='1';
--VGA_SYNC_N<='0';
--PROCESS (CLK143)
--begin
--if rising_edge(clk143)then
--------------double flop sync----------------------
--	RAMADDR2GR_sync0<=RAMADDR2GR;
--	RAMADDR2GR_sync1<=RAMADDR2GR_sync0;
--	RAMADDR2_bin<=gray_to_bin(RAMADDR2GR_sync1);
--   NEXTFRAME(1)<=NEXTFRAME(0);
--	NEXTFRAME(2)<=NEXTFRAME(1);
--
--
--RAMADDR1GR<=bin_to_gray(std_logic_vector(to_unsigned(RAMADDR1,9)));
------------------------------------------------------
--	case BUFF_CTRL is
--		when st0=>------------write image to  SDRAM     
--		if (SDRAM_WAIT='0')then	
--		    SDRAM_WE_N<='0';
--			 SDRAM_RE_N<='1';
--------------------------circle generation------------------
--			if(Xpos<799)then
--				Xpos<=Xpos+1;
--				else
--				Xpos<=0;
--				  if(Ypos<599)then
--				  Ypos<=Ypos+1;
--				  else
--				  Ypos<=0;
--				  end if;	  
--			end if;
--				IF((Xpos-to_integer(unsigned(SW)))*(Xpos-to_integer(unsigned(SW)))+(Ypos-300)*(Ypos-300)<40000)THEN
--				test<='0';
--				else
--				test<='1';
--				end if;
--------------------------------------------------------------
--	       SDRAM_WRITEDATA(7 downto 0)<=(others=>test); 
--	       SDRAM_ADDR<=std_logic_vector(unsigned(SDRAM_ADDR)+1);	
--
--		end if;	
--		
--      if(to_integer(unsigned(SDRAM_ADDR))>(800*600-1) )then-----800x600 resolution
--		   RAMADDR1<=0;
--			BUFF_WAIT<='0';
--			RAMFULL_POINTER<=10;----------min. value 2
--		   BUFF_CTRL<=st1;
--			SDRAM_ADDR<=(others=>'0');
--		end if;
--		
--		when st1=>-----------write from SDRAM to BUFFER
--		      SDRAM_WE_N<='1';
--            RAMWE1<=SDRAM_RDVAL;
--			IF(BUFF_WAIT='0')then
--					 SDRAM_RE_N<='0';
--					   ------------if no wait request is issued and read enable------
--		            IF(SDRAM_WAIT='0' and SDRAM_RE_N='0')THEN	
--							IF(RAMFULL_POINTER<511)then-----move full pointer
--								RAMFULL_POINTER<=RAMFULL_POINTER+1;
--								else
--								RAMFULL_POINTER<=0;
--							end if;			
--			            SDRAM_ADDR<=std_logic_vector(unsigned(SDRAM_ADDR)+1);		
--	               END IF;
--						-------------check if the buffer is full----------------------
--						IF(to_integer(unsigned(RAMADDR2_bin))=(RAMFULL_POINTER))then
--								VGAFLAG(0)<='1';---------init displaying image
--								SDRAM_RE_N<='1';
--								BUFF_WAIT<='1';
--								IF((RAMADDR2+63)<511)THEN
--									RAMRESTART_POINTER<=to_integer(unsigned(RAMADDR2_bin))+63;
--									ELSE
--									RAMRESTART_POINTER<=to_integer(unsigned(RAMADDR2_bin))+63-511;
--								END IF;
--						end if;
--			END IF;
--			    	RAMIN1<=SDRAM_READDATA(7 downto 0);	
--					------------while data is avalable, write to buffer RAM
--					IF(SDRAM_RDVAL='1')then
--						IF(RAMADDR1<511)then
--						RAMADDR1<=RAMADDR1+1;
--						else
--						RAMADDR1<=0;
--						end if;
--					END IF;
--					-------------------------------refill buffer------------------------
--					     IF(to_integer(unsigned(RAMADDR2_bin))=RAMRESTART_POINTER and BUFF_WAIT='1')then
--						  BUFF_WAIT<='0';		  
--						  end if;
--					-------------------------------end of frame--------------------------
-- 				        IF(NEXTFRAME(2)='1')THEN
--						      xpos<=0;
--	                    	ypos<=0;
--							   BUFF_CTRL<=ST0;
--								VGAFLAG(0)<='0';
--								SDRAM_ADDR<=(others=>'0');
--								------------
--								counter<=0;
--								test<='0';
--							END IF;
--		    
--		when others=>NULL;
--		END CASE;
--
--
--
--
--
--end if;
--end process;
--PROCESS(CLK49_5)
--begin
--if rising_edge(CLK49_5)then
--	 
--RAMADDR2GR<=bin_to_gray(std_logic_vector(to_unsigned(RAMADDR2,9)));
---------------dual clock sync-------------------------
--RAMADDR1GR_sync0<=RAMADDR1GR;
--RAMADDR1GR_sync1<=RAMADDR1GR_sync0;
--VGAFLAG(1)<=VGAFLAG(0);
--VGAFLAG(2)<=VGAFLAG(1);
--
--RAMADDR1_bin<=gray_to_bin(RAMADDR1GR_sync1);
--
--
--    IF(VGAFLAG(2)='1' AND FRAMESTART='1' )THEN-------if buffer is rdy and  begin of new frame, start displaying image
--	 VGABEGIN<='1';
--	 end if;
--	 
--	 IF(FRAMEEND='1' AND VGABEGIN='1')THEN------end of frame
--	 NEXTFRAME(0)<='1';
--	 VGABEGIN<='0';
--	 ELSE
--	 NEXTFRAME(0)<='0';
--	 END IF;
--	
--		IF(ACTVIDEO='1'AND to_integer(unsigned(RAMADDR1_bin))/=RAMADDR2  AND VGABEGIN='1')then----if buffer ia not empty
--			IF(RAMADDR2<511)then
--			RAMADDR2<=RAMADDR2+1;
--			else
--			RAMADDR2<=0;
--			end if;
--		   RED<=RAMOUT2;
--	      GREEN<=RAMOUT2;
--		   BLUE<=RAMOUT2;
--		ELSIF(VGABEGIN='0')THEN---------if buffer not ready
--	   RAMADDR2<=0;
--		BLUE<=(others=>'0');
--		RED<=(others=>'0');
--		GREEN<=(others=>'0');
--		END IF;
--end if;
--end process;
--end main;
