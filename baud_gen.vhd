library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity baud_gen is
    port(
        clk       : in std_logic; -- System clock
        reset     : in std_logic; -- Reset signal
        baud_clk  : out std_logic -- Baud rate clock enable signal
    );
end baud_gen;

architecture behavioral of baud_gen is
    constant baud_rate_div : integer := 434; -- 50 MHz clock / 9600 baud rate
    signal counter         : integer range 0 to baud_rate_div := 0;
    signal baud_tick       : std_logic := '0';
begin
    process(clk, reset)
    begin
        if reset = '1' then
            counter <= 0;
            baud_tick <= '0';
        elsif rising_edge(clk) then
            if counter < baud_rate_div then
                counter <= counter + 1;
            else
                counter <= 0;
                baud_tick <= not baud_tick;
            end if;
        end if;
    end process;

    baud_clk <= baud_tick;
end behavioral;
