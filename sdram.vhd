library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pcg.all;

entity sdram is
    port(
        CLOCK_50: IN STD_LOGIC;
        SW: IN STD_LOGIC_VECTOR(9 downto 0);
        ------------------VGA-Interface---------------------
        VGA_B, VGA_G, VGA_R : OUT STD_LOGIC_VECTOR(7 downto 0);
        VGA_CLK, VGA_BLANK_N, VGA_HS, VGA_VS, VGA_SYNC_N: OUT STD_LOGIC;
        LEDR: OUT STD_LOGIC_VECTOR(9 downto 0);
        KEY: IN STD_LOGIC_VECTOR(3 downto 0);
        --------------------SDRAM---------------------------
        DRAM_ADDR: OUT STD_LOGIC_VECTOR(12 downto 0);
        DRAM_BA: OUT STD_LOGIC_VECTOR(1 downto 0);
        DRAM_CAS_N: OUT STD_LOGIC;
        DRAM_CKE: OUT STD_LOGIC;
        DRAM_CLK: OUT STD_LOGIC;
        DRAM_CS_N: OUT STD_LOGIC;
        DRAM_DQ: INOUT STD_LOGIC_VECTOR(15 downto 0);
        DRAM_RAS_N: OUT STD_LOGIC;
        DRAM_WE_N: OUT STD_LOGIC;
        DRAM_LDQM, DRAM_UDQM: OUT STD_LOGIC
    );
end sdram;

architecture main of sdram is
    TYPE STAGES IS (ST0, ST1);
    SIGNAL BUFF_CTRL: STAGES := ST0;
    -----------------------------test signals----------------------------
    signal counter : integer range 0 to 1000;
    signal test: std_logic := '0';
    signal testdata: std_logic_vector(7 downto 0) := "00000000";
    signal Xpos, Ypos: integer range 0 to 799 := 0;
    -----------------------------sync-----------------------------
    signal BUFF_WAIT: std_logic := '0';
    signal VGAFLAG: std_logic_vector(2 downto 0);
    ---------------------------ram/gray----------------------------
    signal RAMFULL_POINTER: integer range 0 to 511 := 0;
    signal RAMRESTART_POINTER: integer range 0 to 511 := 0;
    signal RAMADDR1GR, RAMADDR2GR: std_logic_vector(8 downto 0) := (others => '0');
    signal RAMADDR1GR_sync0, RAMADDR1GR_sync1, RAMADDR1GR_sync2, RAMADDR1_bin: std_logic_vector(8 downto 0);
    signal RAMADDR2GR_sync0, RAMADDR2GR_sync1, RAMADDR2GR_sync2, RAMADDR2_bin: std_logic_vector(8 downto 0);
    ---------------------------dual ram ----------------------------------
    signal RAMIN1, RAMIN2, RAMOUT1, RAMOUT2: std_logic_vector(7 downto 0);
    signal RAMWE1, RAMWE2: std_logic := '0';
    signal RAMADDR1, RAMADDR2: integer range 0 to 511 := 0;
    --------------------vga----------------------------------
    signal NEXTFRAME: std_logic_vector(2 downto 0) := "000";
    signal FRAMEEND, FRAMESTART: std_logic := '0';
    signal ACTVIDEO: std_logic := '0';
    signal VGABEGIN: std_logic := '0';
    signal RED, GREEN, BLUE: STD_LOGIC_VECTOR(7 downto 0);
    --------------------clock--------------------------------
    SIGNAL CLK143, CLK143_2, CLK49_5: STD_LOGIC;
    --------------------sdram--------------------------------
    SIGNAL SDRAM_ADDR: STD_LOGIC_VECTOR(24 downto 0);
    SIGNAL SDRAM_BE_N: STD_LOGIC_VECTOR(1 downto 0);
    SIGNAL SDRAM_CS: STD_LOGIC;
    SIGNAL SDRAM_RDVAL, SDRAM_WAIT: STD_LOGIC;
    SIGNAL SDRAM_RE_N, SDRAM_WE_N: STD_LOGIC;
    SIGNAL SDRAM_READDATA, SDRAM_WRITEDATA: STD_LOGIC_VECTOR(15 downto 0);
    SIGNAL DRAM_DQM : STD_LOGIC_VECTOR(1 downto 0);

    component true_dual_port_ram_dual_clock is
        port (
            clk_a : in std_logic;
            clk_b : in std_logic;
            addr_a : in natural range 0 to 511;
            addr_b : in natural range 0 to 511;
            data_a : in std_logic_vector(7 downto 0);
            data_b : in std_logic_vector(7 downto 0);
            we_a : in std_logic := '1';
            we_b : in std_logic := '1';
            q_a : out std_logic_vector(7 downto 0);
            q_b : out std_logic_vector(7 downto 0)
        );
    end component true_dual_port_ram_dual_clock;

    component vga is
        port(
            CLK: in std_logic;
            R_OUT, G_OUT, B_OUT: OUT std_logic_vector(7 downto 0);
            R_IN, G_IN, B_IN: IN std_logic_vector(7 downto 0);
            VGAHS, VGAVS: OUT std_logic;
            ACTVID: OUT std_logic;
            VGA_FRAMESTART: out std_logic;
            VGA_FRAMEEND: out std_logic
        );
    end component vga;

    component ramsys is
        port (
            clk_clk : in std_logic := 'X';
            reset_reset_n : in std_logic := 'X';
            clk143_shift_clk : out std_logic;
            clk143_clk : out std_logic;
            clk49_5_clk : out std_logic;
            wire_addr : out std_logic_vector(12 downto 0);
            wire_ba : out std_logic_vector(1 downto 0);
            wire_cas_n : out std_logic;
            wire_cke : out std_logic;
            wire_cs_n : out std_logic;
            wire_dq : inout std_logic_vector(15 downto 0) := (others => 'X');
            wire_dqm : out std_logic_vector(1 downto 0);
            wire_ras_n : out std_logic;
            wire_we_n : out std_logic;
            sdram_address : in std_logic_vector(24 downto 0) := (others => 'X');
            sdram_byteenable_n : in std_logic_vector(1 downto 0) := (others => 'X');
            sdram_chipselect : in std_logic := 'X';
            sdram_writedata : in std_logic_vector(15 downto 0) := (others => 'X');
            sdram_read_n : in std_logic := 'X';
            sdram_write_n : in std_logic := 'X';
            sdram_readdata : out std_logic_vector(15 downto 0);
            sdram_readdatavalid : out std_logic;
            sdram_waitrequest : out std_logic
        );
    end component ramsys;

begin
    u0 : component ramsys
        port map (
            clk_clk => CLOCK_50,
            reset_reset_n => '1',
            clk143_shift_clk => CLK143_2,
            clk143_clk => CLK143,
            clk49_5_clk => CLK49_5,
            wire_addr => DRAM_ADDR,
            wire_ba => DRAM_BA,
            wire_cas_n => DRAM_CAS_N,
            wire_cke => DRAM_CKE,
            wire_cs_n => DRAM_CS_N,
            wire_dq => DRAM_DQ,
            wire_dqm => DRAM_DQM,
            wire_ras_n => DRAM_RAS_N,
            wire_we_n => DRAM_WE_N,
            sdram_address => SDRAM_ADDR,
            sdram_byteenable_n => SDRAM_BE_N,
            sdram_chipselect => SDRAM_CS,
            sdram_writedata => SDRAM_WRITEDATA,
            sdram_read_n => SDRAM_RE_N,
            sdram_write_n => SDRAM_WE_N,
            sdram_readdata => SDRAM_READDATA,
            sdram_readdatavalid => SDRAM_RDVAL,
            sdram_waitrequest => SDRAM_WAIT
        );

    u1 : component vga 
        port map(
            CLK => CLK49_5,
            R_OUT => VGA_R,
            G_OUT => VGA_G,
            B_OUT => VGA_B,
            R_IN => RED,
            G_IN => GREEN,
            B_IN => BLUE,
            VGAHS => VGA_HS,
            VGAVS => VGA_VS,
            ACTVID => ACTVIDEO,
            VGA_FRAMESTART => FRAMESTART,
            VGA_FRAMEEND => FRAMEEND
        );

    u3: component true_dual_port_ram_dual_clock
        port map (
            clk_a => CLK143,
            clk_b => clk49_5,
            addr_a => RAMADDR1,
            addr_b => RAMADDR2,
            data_a => RAMIN1,
            data_b => RAMIN2,
            we_a => RAMWE1,
            we_b => RAMWE2,
            q_a => RAMOUT1,
            q_b => RAMOUT2
        );

    DRAM_LDQM <= DRAM_DQM(0);
    DRAM_UDQM <= DRAM_DQM(1);
    DRAM_CLK <= CLK143_2;
    VGA_CLK <= CLK49_5;
    SDRAM_CS <= '1';
    SDRAM_BE_N <= "00";
    VGA_BLANK_N <= '1';
    VGA_SYNC_N <= '0';

end architecture main;
PROCESS (CLK143)
begin
    if rising_edge(CLK143) then
        --------------double flop sync----------------------
        RAMADDR2GR_sync0 <= RAMADDR2GR;
        RAMADDR2GR_sync1 <= RAMADDR2GR_sync0;
        RAMADDR2_bin <= gray_to_bin(RAMADDR2GR_sync1);
        NEXTFRAME(1) <= NEXTFRAME(0);
        NEXTFRAME(2) <= NEXTFRAME(1);

        RAMADDR1GR <= bin_to_gray(std_logic_vector(to_unsigned(RAMADDR1, 9)));
        ------------------------------------------------------
        case BUFF_CTRL is
            when ST0 => ------------write image to SDRAM     
                if (SDRAM_WAIT = '0') then	
                    SDRAM_WE_N <= '0';
                    SDRAM_RE_N <= '1';
                    --------------------------circle generation------------------
                    if (Xpos < 799) then
                        Xpos <= Xpos + 1;
                    else
                        Xpos <= 0;
                        if (Ypos < 599) then
                            Ypos <= Ypos + 1;
                        else
                            Ypos <= 0;
                        end if;	  
                    end if;
                    if ((Xpos - to_integer(unsigned(SW))) * (Xpos - to_integer(unsigned(SW))) + 
                        (Ypos - 300) * (Ypos - 300) < 40000) then
                        test <= '0';
                    else
                        test <= '1';
                    end if;
                    --------------------------------------------------------------
                    SDRAM_WRITEDATA(7 downto 0) <= (others => test); 
                    SDRAM_ADDR <= std_logic_vector(unsigned(SDRAM_ADDR) + 1);	
                end if;	
                if (to_integer(unsigned(SDRAM_ADDR)) > (800 * 600 - 1)) then -----800x600 resolution
                    RAMADDR1 <= 0;
                    BUFF_WAIT <= '0';
                    RAMFULL_POINTER <= 10; ----------min. value 2
                    BUFF_CTRL <= ST1;
                    SDRAM_ADDR <= (others => '0');
                end if;

            when ST1 => -----------write from SDRAM to BUFFER
                SDRAM_WE_N <= '1';
                RAMWE1 <= SDRAM_RDVAL;
                if (BUFF_WAIT = '0') then
                    SDRAM_RE_N <= '0';
                    ------------if no wait request is issued and read enable------
                    if (SDRAM_WAIT = '0' and SDRAM_RE_N = '0') then	
                        if (RAMFULL_POINTER < 511) then -----move full pointer
                            RAMFULL_POINTER <= RAMFULL_POINTER + 1;
                        else
                            RAMFULL_POINTER <= 0;
                        end if;			
                        SDRAM_ADDR <= std_logic_vector(unsigned(SDRAM_ADDR) + 1);		
                    end if;
                    -------------check if the buffer is full----------------------
                    if (to_integer(unsigned(RAMADDR2_bin)) = RAMFULL_POINTER) then
                        VGAFLAG(0) <= '1'; ---------init displaying image
                        SDRAM_RE_N <= '1';
                        BUFF_WAIT <= '1';
                        if ((RAMADDR2 + 63) < 511) then
                            RAMRESTART_POINTER <= to_integer(unsigned(RAMADDR2_bin)) + 63;
                        else
                            RAMRESTART_POINTER <= to_integer(unsigned(RAMADDR2_bin)) + 63 - 511;
                        end if;
                    end if;
                end if;
                RAMIN1 <= SDRAM_READDATA(7 downto 0);	
                ------------while data is available, write to buffer RAM
                if (SDRAM_RDVAL = '1') then
                    if (RAMADDR1 < 511) then
                        RAMADDR1 <= RAMADDR1 + 1;
                    else
                        RAMADDR1 <= 0;
                    end if;
                end if;
                -------------------------------refill buffer------------------------
                if (to_integer(unsigned(RAMADDR2_bin)) = RAMRESTART_POINTER and BUFF_WAIT = '1') then
                    BUFF_WAIT <= '0';		  
                end if;
                -------------------------------end of frame--------------------------
                if (NEXTFRAME(2) = '1') then
                    Xpos <= 0;
                    Ypos <= 0;
                    BUFF_CTRL <= ST0;
                    VGAFLAG(0) <= '0';
                    SDRAM_ADDR <= (others => '0');
                    ------------
                    counter <= 0;
                    test <= '0';
                end if;
            when others =>
                NULL;
        end case;
    end if;
end process;

PROCESS (CLK49_5)
begin
    if rising_edge(CLK49_5) then
        RAMADDR2GR <= bin_to_gray(std_logic_vector(to_unsigned(RAMADDR2, 9)));
        ---------------dual clock sync-------------------------
        RAMADDR1GR_sync0 <= RAMADDR1GR;
        RAMADDR1GR_sync1 <= RAMADDR1GR_sync0;
        VGAFLAG(1) <= VGAFLAG(0);
        VGAFLAG(2) <= VGAFLAG(1);

        RAMADDR1_bin <= gray_to_bin(RAMADDR1GR_sync1);

        if (VGAFLAG(2) = '1' and FRAMESTART = '1') then -------if buffer is ready and  begin of new frame, start displaying image
            VGABEGIN <= '1';
        end if;

        if (FRAMEEND = '1' and VGABEGIN = '1') then ------end of frame
            NEXTFRAME(0) <= '1';
            VGABEGIN <= '0';
        else
            NEXTFRAME(0) <= '0';
        end if;

        if (ACTVIDEO = '1' and to_integer(unsigned(RAMADDR1_bin)) /= RAMADDR2 and VGABEGIN = '1') then ----if buffer is not empty
            if (RAMADDR2 < 511) then
                RAMADDR2 <= RAMADDR2 + 1;
            else
                RAMADDR2 <= 0;
            end if;
            RED <= RAMOUT2;
            GREEN <= RAMOUT2;
            BLUE <= RAMOUT2;
        elsif (VGABEGIN = '0') then ---------if buffer not ready
            RAMADDR2 <= 0;
            BLUE <= (others => '0');
            RED <= (others => '0');
            GREEN <= (others => '0');
        end if;
    end if;
end process;

        end main
            
