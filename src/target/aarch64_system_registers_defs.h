static const struct aarch64_system_register aarch64_system_registers[] = {
	{
		"OSDTRRX_EL1",
		"OS Lock Data Transfer Register, Receive",
		"10", "000", "0000", "0000", "010",
		0x8002,
		{
			{"None", 31, 0},
		}
	},
	{
		"MDCCINT_EL1",
		"Monitor DCC Interrupt Enable Register",
		"10", "000", "0000", "0000", "000",
		0x8010,
		{
			{"RX", 30, 30},
			{"TX", 29, 29},
		}
	},
	{
		"MDSCR_EL1",
		"Monitor Debug System Control Register",
		"10", "000", "0000", "0000", "010",
		0x8012,
		{
			{"TFO", 31, 31},
			{"RXfull", 30, 30},
			{"TXfull", 29, 29},
			{"RXO", 27, 27},
			{"TXU", 26, 26},
			{"INTdis", 23, 22},
			{"TDA", 21, 21},
			{"SC2", 19, 19},
			{"0", 18, 16},
			{"MDE", 15, 15},
			{"HDE", 14, 14},
			{"KDE", 13, 13},
			{"TDCC", 12, 12},
			{"ERR", 6, 6},
			{"SS", 0, 0},
		}
	},
	{
		"OSDTRTX_EL1",
		"OS Lock Data Transfer Register, Transmit",
		"10", "000", "0000", "0000", "010",
		0x801a,
		{
			{"None", 31, 0},
		}
	},
	{
		"OSECCR_EL1",
		"OS Lock Exception Catch Control Register",
		"10", "000", "0000", "0100", "010",
		0x8032,
		{
			{"EDECCR", 31, 0},
		}
	},
	{
		"DBGBVR0_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x8004,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR1_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x8044,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR2_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x8024,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR3_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x8064,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR4_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x8014,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR5_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x8054,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR6_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x8034,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR7_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x8074,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR8_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x800c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR9_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x804c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR10_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x802c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR11_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x806c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR12_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x801c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR13_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0000", "000",
		0x805c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR14_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x803c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBVR15_EL1",
		"Debug Breakpoint Value Registers",
		"10", "000", "0000", "0100", "000",
		0x807c,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGBCR0_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x8005,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR1_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x8045,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR2_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x8025,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR3_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x8065,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR4_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x8015,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR5_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x8055,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR6_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x8035,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR7_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x8075,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR8_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x800d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR9_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x804d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR10_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x802d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR11_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x806d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR12_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x801d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR13_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0000", "001",
		0x805d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR14_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x803d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGBCR15_EL1",
		"Debug Breakpoint Control Registers",
		"10", "000", "0000", "0100", "001",
		0x807d,
		{
			{"BT", 23, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 8, 5},
			{"PMC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWVR0_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x8006,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR1_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x8046,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR2_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x8026,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR3_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x8066,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR4_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x8016,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR5_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x8056,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR6_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x8036,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR7_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x8076,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR8_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x800e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR9_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x804e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR10_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x802e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR11_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x806e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR12_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x801e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR13_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0000", "010",
		0x805e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR14_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x803e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWVR15_EL1",
		"Debug Watchpoint Value Registers",
		"10", "000", "0000", "0100", "010",
		0x807e,
		{
			{"RESS[14:4]", 63, 53},
			{"VA[52:49]", 52, 49},
			{"RESS[3:0]", 52, 49},
			{"VA[48:2]", 48, 2},
		}
	},
	{
		"DBGWCR0_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x8007,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR1_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x8047,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR2_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x8027,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR3_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x8067,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR4_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x8017,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR5_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x8057,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR6_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x8037,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR7_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x8077,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR8_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x800f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR9_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x804f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR10_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x802f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR11_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x806f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR12_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x801f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR13_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0000", "011",
		0x805f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR14_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x803f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"DBGWCR15_EL1",
		"Debug Watchpoint Control Registers",
		"10", "000", "0000", "0100", "011",
		0x807f,
		{
			{"MASK", 28, 24},
			{"WT", 20, 20},
			{"LBN", 19, 16},
			{"SSC", 15, 14},
			{"HMC", 13, 13},
			{"BAS", 12, 5},
			{"LSC", 4, 3},
			{"PAC", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"MDRAR_EL1",
		"Monitor Debug ROM Address Register",
		"10", "000", "0000", "0000", "000",
		0x8080,
		{
			{"ROMADDR[51:48]", 51, 48},
			{"ROMADDR[47:12]", 47, 12},
			{"Valid", 1, 0},
		}
	},
	{
		"OSLAR_EL1",
		"OS Lock Access Register",
		"10", "000", "0000", "0000", "000",
		0x8084,
		{
			{"OSLK", 0, 0},
		}
	},
	{
		"OSLSR_EL1",
		"OS Lock Status Register",
		"10", "000", "0000", "0000", "000",
		0x808c,
		{
			{"OSLM[1]", 3, 3},
			{"nTT", 2, 2},
			{"OSLK", 1, 1},
			{"OSLM[0]", 0, 0},
		}
	},
	{
		"OSDLR_EL1",
		"OS Double Lock Register",
		"10", "000", "0000", "0000", "000",
		0x809c,
		{
			{"DLK", 0, 0},
			{"0", 0, 0},
		}
	},
	{
		"DBGPRCR_EL1",
		"Debug Power Control Register",
		"10", "000", "0000", "0100", "000",
		0x80a4,
		{
			{"CORENPDRQ", 0, 0},
		}
	},
	{
		"DBGCLAIMSET_EL1",
		"Debug Claim Tag Set register",
		"10", "000", "0100", "0000", "010",
		0x83c6,
		{
			{"0", 31, 8},
			{"CLAIM", 7, 0},
		}
	},
	{
		"DBGCLAIMCLR_EL1",
		"Debug Claim Tag Clear register",
		"10", "000", "0100", "0000", "010",
		0x83ce,
		{
			{"0", 31, 8},
			{"CLAIM", 7, 0},
		}
	},
	{
		"DBGAUTHSTATUS_EL1",
		"Debug Authentication Status register",
		"10", "000", "0100", "0100", "010",
		0x83f6,
		{
			{"SNID", 7, 6},
			{"SNID", 7, 6},
			{"SID", 5, 4},
			{"NSNID", 3, 2},
			{"NSNID", 3, 2},
			{"NSID", 1, 0},
		}
	},
	{
		"MDCCSR_EL0",
		"Monitor DCC Status Register",
		"10", "011", "0000", "0000", "000",
		0x9808,
		{
			{"RXfull", 30, 30},
			{"TXfull", 29, 29},
			{"0", 18, 15},
			{"0", 12, 12},
			{"0", 5, 2},
		}
	},
	{
		"DBGDTR_EL0",
		"Debug Data Transfer Register, half-duplex",
		"10", "011", "0000", "0100", "000",
		0x9820,
		{
			{"HighWord", 63, 32},
			{"LowWord", 31, 0},
		}
	},
	{
		"DBGDTRRX_EL0",
		"Debug Data Transfer Register, Receive",
		"10", "011", "0000", "0100", "000",
		0x9828,
		{
			{"None", 31, 0},
		}
	},
	{
		"DBGDTRTX_EL0",
		"Debug Data Transfer Register, Transmit",
		"10", "011", "0000", "0100", "000",
		0x9828,
		{
			{"None", 31, 0},
		}
	},
	{
		"DBGVCR32_EL2",
		"Debug Vector Catch Register",
		"10", "000", "0000", "0100", "000",
		0xa038,
		{
			{"NSF", 31, 31},
			{"NSI", 30, 30},
			{"NSD", 28, 28},
			{"NSP", 27, 27},
			{"NSS", 26, 26},
			{"NSU", 25, 25},
			{"SF", 7, 7},
			{"SI", 6, 6},
			{"SD", 4, 4},
			{"SP", 3, 3},
			{"SS", 2, 2},
			{"SU", 1, 1},
		}
	},
	{
		"MIDR_EL1",
		"Main ID Register",
		"10", "000", "0000", "0000", "000",
		0xc000,
		{
			{"Implementer", 31, 24},
			{"Variant", 23, 20},
			{"Architecture", 19, 16},
			{"PartNum", 15, 4},
			{"Revision", 3, 0},
		}
	},
	{
		"MPIDR_EL1",
		"Multiprocessor Affinity Register",
		"10", "000", "0000", "0000", "001",
		0xc005,
		{
			{"Aff3", 39, 32},
			{"U", 30, 30},
			{"MT", 24, 24},
			{"Aff2", 23, 16},
			{"Aff1", 15, 8},
			{"Aff0", 7, 0},
		}
	},
	{
		"REVIDR_EL1",
		"Revision ID Register",
		"10", "000", "0000", "0000", "010",
		0xc006,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"ID_PFR0_EL1",
		"AArch32 Processor Feature Register 0",
		"10", "000", "0000", "0000", "000",
		0xc008,
		{
			{"RAS", 31, 28},
			{"DIT", 27, 24},
			{"AMU", 23, 20},
			{"CSV2", 19, 16},
			{"State3", 15, 12},
			{"State2", 11, 8},
			{"State1", 7, 4},
			{"State0", 3, 0},
		}
	},
	{
		"ID_PFR1_EL1",
		"AArch32 Processor Feature Register 1",
		"10", "000", "0000", "0000", "001",
		0xc009,
		{
			{"GIC", 31, 28},
			{"Virt_frac", 27, 24},
			{"Sec_frac", 23, 20},
			{"GenTimer", 19, 16},
			{"Virtualization", 15, 12},
			{"MProgMod", 11, 8},
			{"Security", 7, 4},
			{"ProgMod", 3, 0},
		}
	},
	{
		"ID_DFR0_EL1",
		"AArch32 Debug Feature Register 0",
		"10", "000", "0000", "0000", "010",
		0xc00a,
		{
			{"TraceFilt", 31, 28},
			{"PerfMon", 27, 24},
			{"MProfDbg", 23, 20},
			{"MMapTrc", 19, 16},
			{"CopTrc", 15, 12},
			{"MMapDbg", 11, 8},
			{"CopSDbg", 7, 4},
			{"CopDbg", 3, 0},
		}
	},
	{
		"ID_AFR0_EL1",
		"AArch32 Auxiliary Feature Register 0",
		"10", "000", "0000", "0000", "011",
		0xc00b,
		{
			{"IMPLEMENTATION DEFINED", 15, 12},
			{"IMPLEMENTATION DEFINED", 11, 8},
			{"IMPLEMENTATION DEFINED", 7, 4},
			{"IMPLEMENTATION DEFINED", 3, 0},
		}
	},
	{
		"ID_MMFR0_EL1",
		"AArch32 Memory Model Feature Register 0",
		"10", "000", "0000", "0000", "000",
		0xc00c,
		{
			{"InnerShr", 31, 28},
			{"FCSE", 27, 24},
			{"AuxReg", 23, 20},
			{"TCM", 19, 16},
			{"ShareLvl", 15, 12},
			{"OuterShr", 11, 8},
			{"PMSA", 7, 4},
			{"VMSA", 3, 0},
		}
	},
	{
		"ID_MMFR1_EL1",
		"AArch32 Memory Model Feature Register 1",
		"10", "000", "0000", "0000", "001",
		0xc00d,
		{
			{"BPred", 31, 28},
			{"L1TstCln", 27, 24},
			{"L1Uni", 23, 20},
			{"L1Hvd", 19, 16},
			{"L1UniSW", 15, 12},
			{"L1HvdSW", 11, 8},
			{"L1UniVA", 7, 4},
			{"L1HvdVA", 3, 0},
		}
	},
	{
		"ID_MMFR2_EL1",
		"AArch32 Memory Model Feature Register 2",
		"10", "000", "0000", "0000", "010",
		0xc00e,
		{
			{"HWAccFlg", 31, 28},
			{"WFIStall", 27, 24},
			{"MemBarr", 23, 20},
			{"UniTLB", 19, 16},
			{"HvdTLB", 15, 12},
			{"L1HvdRng", 11, 8},
			{"L1HvdBG", 7, 4},
			{"L1HvdFG", 3, 0},
		}
	},
	{
		"ID_MMFR3_EL1",
		"AArch32 Memory Model Feature Register 3",
		"10", "000", "0000", "0000", "011",
		0xc00f,
		{
			{"Supersec", 31, 28},
			{"CMemSz", 27, 24},
			{"CohWalk", 23, 20},
			{"PAN", 19, 16},
			{"MaintBcst", 15, 12},
			{"BPMaint", 11, 8},
			{"CMaintSW", 7, 4},
			{"CMaintVA", 3, 0},
		}
	},
	{
		"ID_ISAR0_EL1",
		"AArch32 Instruction Set Attribute Register 0",
		"10", "000", "0000", "0000", "000",
		0xc010,
		{
			{"Divide", 27, 24},
			{"Debug", 23, 20},
			{"Coproc", 19, 16},
			{"CmpBranch", 15, 12},
			{"BitField", 11, 8},
			{"BitCount", 7, 4},
			{"Swap", 3, 0},
		}
	},
	{
		"ID_ISAR1_EL1",
		"AArch32 Instruction Set Attribute Register 1",
		"10", "000", "0000", "0000", "001",
		0xc011,
		{
			{"Jazelle", 31, 28},
			{"Interwork", 27, 24},
			{"Immediate", 23, 20},
			{"IfThen", 19, 16},
			{"Extend", 15, 12},
			{"Except_AR", 11, 8},
			{"Except", 7, 4},
			{"Endian", 3, 0},
		}
	},
	{
		"ID_ISAR2_EL1",
		"AArch32 Instruction Set Attribute Register 2",
		"10", "000", "0000", "0000", "010",
		0xc012,
		{
			{"Reversal", 31, 28},
			{"PSR_AR", 27, 24},
			{"MultU", 23, 20},
			{"MultS", 19, 16},
			{"Mult", 15, 12},
			{"MultiAccessInt", 11, 8},
			{"MemHint", 7, 4},
			{"LoadStore", 3, 0},
		}
	},
	{
		"ID_ISAR3_EL1",
		"AArch32 Instruction Set Attribute Register 3",
		"10", "000", "0000", "0000", "011",
		0xc013,
		{
			{"T32EE", 31, 28},
			{"TrueNOP", 27, 24},
			{"T32Copy", 23, 20},
			{"TabBranch", 19, 16},
			{"SynchPrim", 15, 12},
			{"SVC", 11, 8},
			{"SIMD", 7, 4},
			{"Saturate", 3, 0},
		}
	},
	{
		"ID_ISAR4_EL1",
		"AArch32 Instruction Set Attribute Register 4",
		"10", "000", "0000", "0000", "000",
		0xc014,
		{
			{"SWP_frac", 31, 28},
			{"PSR_M", 27, 24},
			{"SynchPrim_frac", 23, 20},
			{"Barrier", 19, 16},
			{"SMC", 15, 12},
			{"Writeback", 11, 8},
			{"WithShifts", 7, 4},
			{"Unpriv", 3, 0},
		}
	},
	{
		"ID_ISAR5_EL1",
		"AArch32 Instruction Set Attribute Register 5",
		"10", "000", "0000", "0000", "001",
		0xc015,
		{
			{"VCMA", 31, 28},
			{"RDM", 27, 24},
			{"CRC32", 19, 16},
			{"SHA2", 15, 12},
			{"SHA1", 11, 8},
			{"AES", 7, 4},
			{"SEVL", 3, 0},
		}
	},
	{
		"ID_MMFR4_EL1",
		"AArch32 Memory Model Feature Register 4",
		"10", "000", "0000", "0000", "010",
		0xc016,
		{
			{"EVT", 31, 28},
			{"CCIDX", 27, 24},
			{"0", 27, 24},
			{"LSM", 23, 20},
			{"0", 23, 20},
			{"HPDS", 19, 16},
			{"0", 19, 16},
			{"CnP", 15, 12},
			{"0", 15, 12},
			{"XNX", 11, 8},
			{"0", 11, 8},
			{"AC2", 7, 4},
			{"SpecSEI", 3, 0},
			{"SpecSEI", 3, 0},
		}
	},
	{
		"ID_ISAR6_EL1",
		"AArch32 Instruction Set Attribute Register 6",
		"10", "000", "0000", "0000", "011",
		0xc017,
		{
			{"SPECRES", 19, 16},
			{"SB", 15, 12},
			{"FHM", 11, 8},
			{"DP", 7, 4},
			{"JSCVT", 3, 0},
		}
	},
	{
		"MVFR0_EL1",
		"AArch32 Media and VFP Feature Register 0",
		"10", "000", "0000", "0000", "000",
		0xc018,
		{
			{"FPRound", 31, 28},
			{"FPShVec", 27, 24},
			{"FPSqrt", 23, 20},
			{"FPDivide", 19, 16},
			{"FPTrap", 15, 12},
			{"FPDP", 11, 8},
			{"FPSP", 7, 4},
			{"SIMDReg", 3, 0},
		}
	},
	{
		"MVFR1_EL1",
		"AArch32 Media and VFP Feature Register 1",
		"10", "000", "0000", "0000", "001",
		0xc019,
		{
			{"SIMDFMAC", 31, 28},
			{"FPHP", 27, 24},
			{"SIMDHP", 23, 20},
			{"SIMDSP", 19, 16},
			{"SIMDInt", 15, 12},
			{"SIMDLS", 11, 8},
			{"FPDNaN", 7, 4},
			{"FPFtZ", 3, 0},
		}
	},
	{
		"MVFR2_EL1",
		"AArch32 Media and VFP Feature Register 2",
		"10", "000", "0000", "0000", "010",
		0xc01a,
		{
			{"FPMisc", 7, 4},
			{"SIMDMisc", 3, 0},
		}
	},
	{
		"ID_PFR2_EL1",
		"AArch32 Processor Feature Register 2",
		"10", "000", "0000", "0000", "000",
		0xc01c,
		{
			{"RAS_frac", 11, 8},
			{"SSBS", 7, 4},
			{"CSV3", 3, 0},
		}
	},
	{
		"ID_AA64PFR0_EL1",
		"AArch64 Processor Feature Register 0",
		"10", "000", "0000", "0100", "000",
		0xc020,
		{
			{"CSV3", 63, 60},
			{"CSV2", 59, 56},
			{"DIT", 51, 48},
			{"AMU", 47, 44},
			{"MPAM", 43, 40},
			{"SEL2", 39, 36},
			{"SVE", 35, 32},
			{"RAS", 31, 28},
			{"GIC", 27, 24},
			{"AdvSIMD", 23, 20},
			{"FP", 19, 16},
			{"EL3", 15, 12},
			{"EL2", 11, 8},
			{"EL1", 7, 4},
			{"EL0", 3, 0},
		}
	},
	{
		"ID_AA64PFR1_EL1",
		"AArch64 Processor Feature Register 1",
		"10", "000", "0000", "0100", "001",
		0xc021,
		{
			{"RAS_frac", 15, 12},
			{"MTE", 11, 8},
			{"SSBS", 7, 4},
			{"BT", 3, 0},
		}
	},
	{
		"ID_AA64ZFR0_EL1",
		"SVE Feature ID register 0",
		"10", "000", "0000", "0100", "000",
		0xc024,
		{
			{"SVEver", 3, 0},
		}
	},
	{
		"ID_AA64DFR0_EL1",
		"AArch64 Debug Feature Register 0",
		"10", "000", "0000", "0100", "000",
		0xc028,
		{
			{"TraceFilt", 43, 40},
			{"DoubleLock", 39, 36},
			{"PMSVer", 35, 32},
			{"CTX_CMPs", 31, 28},
			{"WRPs", 23, 20},
			{"BRPs", 15, 12},
			{"PMUVer", 11, 8},
			{"TraceVer", 7, 4},
			{"DebugVer", 3, 0},
		}
	},
	{
		"ID_AA64DFR1_EL1",
		"AArch64 Debug Feature Register 1",
		"10", "000", "0000", "0100", "001",
		0xc029,
		{
		}
	},
	{
		"ID_AA64AFR0_EL1",
		"AArch64 Auxiliary Feature Register 0",
		"10", "000", "0000", "0100", "000",
		0xc02c,
		{
			{"IMPLEMENTATION DEFINED", 31, 28},
			{"IMPLEMENTATION DEFINED", 27, 24},
			{"IMPLEMENTATION DEFINED", 23, 20},
			{"IMPLEMENTATION DEFINED", 19, 16},
			{"IMPLEMENTATION DEFINED", 15, 12},
			{"IMPLEMENTATION DEFINED", 11, 8},
			{"IMPLEMENTATION DEFINED", 7, 4},
			{"IMPLEMENTATION DEFINED", 3, 0},
		}
	},
	{
		"ID_AA64AFR1_EL1",
		"AArch64 Auxiliary Feature Register 1",
		"10", "000", "0000", "0100", "001",
		0xc02d,
		{
		}
	},
	{
		"ID_AA64ISAR0_EL1",
		"AArch64 Instruction Set Attribute Register 0",
		"10", "000", "0000", "0100", "000",
		0xc030,
		{
			{"RNDR", 63, 60},
			{"TLB", 59, 56},
			{"TS", 55, 52},
			{"FHM", 51, 48},
			{"DP", 47, 44},
			{"SM4", 43, 40},
			{"SM3", 39, 36},
			{"SHA3", 35, 32},
			{"RDM", 31, 28},
			{"Atomic", 23, 20},
			{"CRC32", 19, 16},
			{"SHA2", 15, 12},
			{"SHA1", 11, 8},
			{"AES", 7, 4},
		}
	},
	{
		"ID_AA64ISAR1_EL1",
		"AArch64 Instruction Set Attribute Register 1",
		"10", "000", "0000", "0100", "001",
		0xc031,
		{
			{"SPECRES", 43, 40},
			{"SB", 39, 36},
			{"FRINTTS", 35, 32},
			{"GPI", 31, 28},
			{"GPA", 27, 24},
			{"LRCPC", 23, 20},
			{"LRCPC", 23, 20},
			{"FCMA", 19, 16},
			{"JSCVT", 15, 12},
			{"API", 11, 8},
			{"APA", 7, 4},
			{"DPB", 3, 0},
		}
	},
	{
		"ID_AA64MMFR0_EL1",
		"AArch64 Memory Model Feature Register 0",
		"10", "000", "0000", "0100", "000",
		0xc038,
		{
			{"ExS", 47, 44},
			{"TGran4_2", 43, 40},
			{"TGran64_2", 39, 36},
			{"TGran16_2", 35, 32},
			{"TGran4", 31, 28},
			{"TGran64", 27, 24},
			{"TGran16", 23, 20},
			{"BigEndEL0", 19, 16},
			{"SNSMem", 15, 12},
			{"BigEnd", 11, 8},
			{"ASIDBits", 7, 4},
			{"PARange", 3, 0},
		}
	},
	{
		"ID_AA64MMFR1_EL1",
		"AArch64 Memory Model Feature Register 1",
		"10", "000", "0000", "0100", "001",
		0xc039,
		{
			{"XNX", 31, 28},
			{"SpecSEI", 27, 24},
			{"SpecSEI", 27, 24},
			{"PAN", 23, 20},
			{"LO", 19, 16},
			{"HPDS", 15, 12},
			{"VH", 11, 8},
			{"VMIDBits", 7, 4},
			{"HAFDBS", 3, 0},
		}
	},
	{
		"ID_AA64MMFR2_EL1",
		"AArch64 Memory Model Feature Register 2",
		"10", "000", "0000", "0100", "010",
		0xc03a,
		{
			{"E0PD", 63, 60},
			{"EVT", 59, 56},
			{"BBM", 55, 52},
			{"TTL", 51, 48},
			{"FWB", 43, 40},
			{"IDS", 39, 36},
			{"AT", 35, 32},
			{"ST", 31, 28},
			{"NV", 27, 24},
			{"NV", 27, 24},
			{"CCIDX", 23, 20},
			{"VARange", 19, 16},
			{"IESB", 15, 12},
			{"LSM", 11, 8},
			{"UAO", 7, 4},
			{"CnP", 3, 0},
		}
	},
	{
		"SCTLR_EL1",
		"System Control Register (EL1)",
		"10", "000", "0000", "0000", "000",
		0xc080,
		{
			{"DSSBS", 44, 44},
			{"ATA", 43, 43},
			{"ATA0", 42, 42},
			{"TCF", 41, 40},
			{"TCF0", 39, 38},
			{"ITFSB", 37, 37},
			{"BT1", 36, 36},
			{"BT0", 35, 35},
			{"EnIA", 31, 31},
			{"EnIB", 30, 30},
			{"LSMAOE", 29, 29},
			{"nTLSMD", 28, 28},
			{"EnDA", 27, 27},
			{"UCI", 26, 26},
			{"EE", 25, 25},
			{"E0E", 24, 24},
			{"SPAN", 23, 23},
			{"EIS", 22, 22},
			{"IESB", 21, 21},
			{"TSCXT", 20, 20},
			{"WXN", 19, 19},
			{"nTWE", 18, 18},
			{"nTWE", 18, 18},
			{"nTWI", 16, 16},
			{"nTWI", 16, 16},
			{"UCT", 15, 15},
			{"DZE", 14, 14},
			{"EnDB", 13, 13},
			{"I", 12, 12},
			{"EOS", 11, 11},
			{"EnRCTX", 10, 10},
			{"UMA", 9, 9},
			{"SED", 8, 8},
			{"ITD", 7, 7},
			{"nAA", 6, 6},
			{"CP15BEN", 5, 5},
			{"SA0", 4, 4},
			{"SA", 3, 3},
			{"C", 2, 2},
			{"A", 1, 1},
			{"M", 0, 0},
		}
	},
	{
		"ACTLR_EL1",
		"Auxiliary Control Register (EL1)",
		"10", "000", "0000", "0000", "001",
		0xc081,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"CPACR_EL1",
		"Architectural Feature Access Control Register",
		"10", "000", "0000", "0000", "010",
		0xc082,
		{
			{"TTA", 28, 28},
			{"FPEN", 21, 20},
			{"ZEN", 17, 16},
		}
	},
	{
		"RGSR_EL1",
		"Random Allocation Tag Seed Register.",
		"10", "000", "0000", "0000", "001",
		0xc085,
		{
			{"SEED", 23, 8},
			{"TAG", 3, 0},
		}
	},
	{
		"GCR_EL1",
		"Tag Control Register.",
		"10", "000", "0000", "0000", "010",
		0xc086,
		{
			{"RRND", 16, 16},
			{"Exclude", 15, 0},
		}
	},
	{
		"ZCR_EL1",
		"SVE Control Register for EL1",
		"10", "000", "0000", "0000", "000",
		0xc090,
		{
			{"0", 8, 4},
			{"LEN", 3, 0},
		}
	},
	{
		"TRFCR_EL1",
		"Trace Filter Control Register (EL1)",
		"10", "000", "0000", "0000", "001",
		0xc091,
		{
			{"TS", 6, 5},
			{"E1TRE", 1, 1},
			{"E0TRE", 0, 0},
		}
	},
	{
		"TTBR0_EL1",
		"Translation Table Base Register 0 (EL1)",
		"10", "000", "0000", "0000", "000",
		0xc100,
		{
			{"ASID", 63, 48},
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"TTBR1_EL1",
		"Translation Table Base Register 1 (EL1)",
		"10", "000", "0000", "0000", "001",
		0xc101,
		{
			{"ASID", 63, 48},
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"TCR_EL1",
		"Translation Control Register (EL1)",
		"10", "000", "0000", "0000", "010",
		0xc102,
		{
			{"TCMA1", 58, 58},
			{"TCMA0", 57, 57},
			{"E0PD1", 56, 56},
			{"E0PD0", 55, 55},
			{"NFD1", 54, 54},
			{"NFD0", 53, 53},
			{"TBID1", 52, 52},
			{"TBID0", 51, 51},
			{"HWU162", 50, 50},
			{"HWU161", 49, 49},
			{"HWU160", 48, 48},
			{"HWU159", 47, 47},
			{"HWU062", 46, 46},
			{"HWU061", 45, 45},
			{"HWU060", 44, 44},
			{"HWU059", 43, 43},
			{"HPD1", 42, 42},
			{"HPD0", 41, 41},
			{"HD", 40, 40},
			{"HA", 39, 39},
			{"TBI1", 38, 38},
			{"TBI0", 37, 37},
			{"AS", 36, 36},
			{"IPS", 34, 32},
			{"TG1", 31, 30},
			{"SH1", 29, 28},
			{"ORGN1", 27, 26},
			{"IRGN1", 25, 24},
			{"EPD1", 23, 23},
			{"A1", 22, 22},
			{"T1SZ", 21, 16},
			{"TG0", 15, 14},
			{"SH0", 13, 12},
			{"ORGN0", 11, 10},
			{"IRGN0", 9, 8},
			{"EPD0", 7, 7},
			{"T0SZ", 5, 0},
		}
	},
	{
		"APIAKeyLo_EL1",
		"Pointer Authentication Key A for Instruction (bits[63:0])",
		"10", "000", "0000", "0000", "000",
		0xc108,
		{
			{"None", 63, 0},
		}
	},
	{
		"APIAKeyHi_EL1",
		"Pointer Authentication Key A for Instruction (bits[127:64])",
		"10", "000", "0000", "0000", "001",
		0xc109,
		{
			{"None", 63, 0},
		}
	},
	{
		"APIBKeyLo_EL1",
		"Pointer Authentication Key B for Instruction (bits[63:0])",
		"10", "000", "0000", "0000", "010",
		0xc10a,
		{
			{"None", 63, 0},
		}
	},
	{
		"APIBKeyHi_EL1",
		"Pointer Authentication Key B for Instruction (bits[127:64])",
		"10", "000", "0000", "0000", "011",
		0xc10b,
		{
			{"None", 63, 0},
		}
	},
	{
		"APDAKeyLo_EL1",
		"Pointer Authentication Key A for Data (bits[63:0])",
		"10", "000", "0000", "0000", "000",
		0xc110,
		{
			{"None", 63, 0},
		}
	},
	{
		"APDAKeyHi_EL1",
		"Pointer Authentication Key A for Data (bits[127:64])",
		"10", "000", "0000", "0000", "001",
		0xc111,
		{
			{"None", 63, 0},
		}
	},
	{
		"APDBKeyLo_EL1",
		"Pointer Authentication Key B for Data (bits[63:0])",
		"10", "000", "0000", "0000", "010",
		0xc112,
		{
			{"None", 63, 0},
		}
	},
	{
		"APDBKeyHi_EL1",
		"Pointer Authentication Key B for Data (bits[127:64])",
		"10", "000", "0000", "0000", "011",
		0xc113,
		{
			{"None", 63, 0},
		}
	},
	{
		"APGAKeyLo_EL1",
		"Pointer Authentication Key A for Code (bits[63:0])",
		"10", "000", "0000", "0000", "000",
		0xc118,
		{
			{"None", 63, 0},
		}
	},
	{
		"APGAKeyHi_EL1",
		"Pointer Authentication Key A for Code (bits[127:64])",
		"10", "000", "0000", "0000", "001",
		0xc119,
		{
			{"None", 63, 0},
		}
	},
	{
		"SPSR_EL1",
		"Saved Program Status Register (EL1)",
		"10", "000", "0100", "0000", "000",
		0xc200,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"DIT", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"SS", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4]", 4, 4},
			{"M[3:0]", 3, 0},
		}
	},
	{
		"ELR_EL1",
		"Exception Link Register (EL1)",
		"10", "000", "0100", "0000", "001",
		0xc201,
		{
			{"None", 63, 0},
		}
	},
	{
		"SP_EL0",
		"Stack Pointer (EL0)",
		"10", "000", "0100", "0000", "000",
		0xc208,
		{
			{"None", 63, 0},
		}
	},
	{
		"SPSel",
		"Stack Pointer Select",
		"10", "000", "0100", "0000", "000",
		0xc210,
		{
			{"SP", 0, 0},
		}
	},
	{
		"CurrentEL",
		"Current Exception Level",
		"10", "000", "0100", "0000", "010",
		0xc212,
		{
			{"EL", 3, 2},
		}
	},
	{
		"PAN",
		"Privileged Access Never",
		"10", "000", "0100", "0000", "011",
		0xc213,
		{
			{"PAN", 22, 22},
		}
	},
	{
		"UAO",
		"User Access Override",
		"10", "000", "0100", "0000", "000",
		0xc214,
		{
			{"UAO", 23, 23},
		}
	},
	{
		"ICC_PMR_EL1",
		"Interrupt Controller Interrupt Priority Mask Register",
		"10", "000", "0100", "0100", "000",
		0xc230,
		{
			{"Priority", 7, 0},
		}
	},
	{
		"AFSR0_EL1",
		"Auxiliary Fault Status Register 0 (EL1)",
		"10", "000", "0100", "0000", "000",
		0xc288,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"AFSR1_EL1",
		"Auxiliary Fault Status Register 1 (EL1)",
		"10", "000", "0100", "0000", "001",
		0xc289,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"ESR_EL1",
		"Exception Syndrome Register (EL1)",
		"10", "000", "0100", "0000", "000",
		0xc290,
		{
			{"EC", 31, 26},
			{"IL", 25, 25},
			{"ISS", 24, 0},
		}
	},
	{
		"ERRIDR_EL1",
		"Error Record ID Register",
		"10", "000", "0100", "0000", "000",
		0xc298,
		{
			{"NUM", 15, 0},
		}
	},
	{
		"ERRSELR_EL1",
		"Error Record Select Register",
		"10", "000", "0100", "0000", "001",
		0xc299,
		{
			{"SEL", 15, 0},
		}
	},
	{
		"ERXFR_EL1",
		"Selected Error Record Feature Register",
		"10", "000", "0100", "0100", "000",
		0xc2a0,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXCTLR_EL1",
		"Selected Error Record Control Register",
		"10", "000", "0100", "0100", "001",
		0xc2a1,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXSTATUS_EL1",
		"Selected Error Record Primary Status Register",
		"10", "000", "0100", "0100", "010",
		0xc2a2,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXADDR_EL1",
		"Selected Error Record Address Register",
		"10", "000", "0100", "0100", "011",
		0xc2a3,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXPFGF_EL1",
		"Selected Pseudo-fault Generation Feature Register",
		"10", "000", "0100", "0100", "000",
		0xc2a4,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXPFGCTL_EL1",
		"Selected Pseudo-fault Generation Control Register",
		"10", "000", "0100", "0100", "001",
		0xc2a5,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXPFGCDN_EL1",
		"Selected Pseudo-fault Generation Countdown Register",
		"10", "000", "0100", "0100", "010",
		0xc2a6,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXMISC0_EL1",
		"Selected Error Record Miscellaneous Register 0",
		"10", "000", "0100", "0100", "000",
		0xc2a8,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXMISC1_EL1",
		"Selected Error Record Miscellaneous Register 1",
		"10", "000", "0100", "0100", "001",
		0xc2a9,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXMISC2_EL1",
		"Selected Error Record Miscellaneous Register 2",
		"10", "000", "0100", "0100", "010",
		0xc2aa,
		{
			{"None", 63, 0},
		}
	},
	{
		"ERXMISC3_EL1",
		"Selected Error Record Miscellaneous Register 3",
		"10", "000", "0100", "0100", "011",
		0xc2ab,
		{
			{"None", 63, 0},
		}
	},
	{
		"FAR_EL1",
		"Fault Address Register (EL1)",
		"10", "000", "0100", "0000", "000",
		0xc300,
		{
			{"None", 63, 0},
		}
	},
	{
		"TFSR_EL1",
		"Tag Fail Status Register (EL1).",
		"10", "000", "0100", "0100", "000",
		0xc328,
		{
			{"TF1", 1, 1},
			{"TF0", 0, 0},
		}
	},
	{
		"TFSRE0_EL1",
		"Tag Fail Status Register (EL0).",
		"10", "000", "0100", "0100", "001",
		0xc331,
		{
			{"TF1", 1, 1},
			{"TF0", 0, 0},
		}
	},
	{
		"PAR_EL1",
		"Physical Address Register",
		"10", "000", "0100", "0100", "000",
		0xc3a0,
		{
			{"ATTR", 63, 56},
			{"PA[51:48]", 51, 48},
			{"PA[47:12]", 47, 12},
			{"IMPLEMENTATION DEFINED", 10, 10},
			{"NS", 9, 9},
			{"SH", 8, 7},
			{"F", 0, 0},
		}
	},
	{
		"PMSCR_EL1",
		"Statistical Profiling Control Register (EL1)",
		"10", "000", "0000", "0000", "000",
		0xc4c8,
		{
			{"PCT", 6, 6},
			{"TS", 5, 5},
			{"PA", 4, 4},
			{"CX", 3, 3},
			{"E1SPE", 1, 1},
			{"E0SPE", 0, 0},
		}
	},
	{
		"PMSICR_EL1",
		"Sampling Interval Counter Register",
		"10", "000", "0000", "0000", "010",
		0xc4ca,
		{
			{"ECOUNT", 63, 56},
			{"COUNT", 31, 0},
		}
	},
	{
		"PMSIRR_EL1",
		"Sampling Interval Reload Register",
		"10", "000", "0000", "0000", "011",
		0xc4cb,
		{
			{"INTERVAL", 31, 8},
			{"RND", 0, 0},
		}
	},
	{
		"PMSFCR_EL1",
		"Sampling Filter Control Register",
		"10", "000", "0000", "0000", "000",
		0xc4cc,
		{
			{"ST", 18, 18},
			{"LD", 17, 17},
			{"B", 16, 16},
			{"FL", 2, 2},
			{"FT", 1, 1},
			{"FE", 0, 0},
		}
	},
	{
		"PMSEVFR_EL1",
		"Sampling Event Filter Register",
		"10", "000", "0000", "0000", "001",
		0xc4cd,
		{
			{"E[<z>]", 63, 48},
			{"0", 47, 32},
			{"E[<y>]", 31, 24},
			{"0", 23, 19},
			{"E[18]", 18, 18},
			{"0", 18, 18},
			{"E[17]", 17, 17},
			{"0", 17, 17},
			{"0", 16, 16},
			{"E[<x>]", 15, 12},
			{"E[11]", 11, 11},
			{"0", 11, 11},
			{"0", 10, 8},
			{"E[7]", 7, 7},
			{"0", 6, 6},
			{"E[5]", 5, 5},
			{"0", 4, 4},
			{"E[3]", 3, 3},
			{"0", 2, 2},
			{"E[1]", 1, 1},
			{"UNKNOWN", 1, 1},
			{"0", 0, 0},
		}
	},
	{
		"PMSLATFR_EL1",
		"Sampling Latency Filter Register",
		"10", "000", "0000", "0000", "010",
		0xc4ce,
		{
			{"MINLAT", 11, 0},
		}
	},
	{
		"PMSIDR_EL1",
		"Sampling Profiling ID Register",
		"10", "000", "0000", "0000", "011",
		0xc4cf,
		{
			{"CountSize", 19, 16},
			{"MaxSize", 15, 12},
			{"Interval", 11, 8},
			{"ERnd", 5, 5},
			{"LDS", 4, 4},
			{"ArchInst", 3, 3},
			{"FL", 2, 2},
			{"FT", 1, 1},
			{"FE", 0, 0},
		}
	},
	{
		"PMBLIMITR_EL1",
		"Profiling Buffer Limit Address Register",
		"10", "000", "0000", "0000", "000",
		0xc4d0,
		{
			{"LIMIT", 63, 12},
			{"FM", 2, 1},
			{"E", 0, 0},
		}
	},
	{
		"PMBPTR_EL1",
		"Profiling Buffer Write Pointer Register",
		"10", "000", "0000", "0000", "001",
		0xc4d1,
		{
			{"PTR", 63, 0},
		}
	},
	{
		"PMBSR_EL1",
		"Profiling Buffer Status/syndrome Register",
		"10", "000", "0000", "0000", "011",
		0xc4d3,
		{
			{"EC", 31, 26},
			{"DL", 19, 19},
			{"EA", 18, 18},
			{"S", 17, 17},
			{"COLL", 16, 16},
			{"MSS", 15, 0},
		}
	},
	{
		"PMBIDR_EL1",
		"Profiling Buffer ID Register",
		"10", "000", "0000", "0000", "011",
		0xc4d7,
		{
			{"F", 5, 5},
			{"P", 4, 4},
			{"P", 4, 4},
			{"Align", 3, 0},
		}
	},
	{
		"PMINTENSET_EL1",
		"Performance Monitors Interrupt Enable Set register",
		"10", "000", "0000", "0100", "001",
		0xc4f1,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"PMINTENCLR_EL1",
		"Performance Monitors Interrupt Enable Clear register",
		"10", "000", "0000", "0100", "010",
		0xc4f2,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"PMMIR_EL1",
		"Performance Monitors Machine Identification Register",
		"10", "000", "0000", "0100", "010",
		0xc4f6,
		{
			{"SLOTS", 7, 0},
		}
	},
	{
		"MAIR_EL1",
		"Memory Attribute Indirection Register (EL1)",
		"10", "000", "0000", "0000", "000",
		0xc510,
		{
			{"Attr<n>", 63, 0},
		}
	},
	{
		"AMAIR_EL1",
		"Auxiliary Memory Attribute Indirection Register (EL1)",
		"10", "000", "0000", "0000", "000",
		0xc518,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"LORSA_EL1",
		"LORegion Start Address (EL1)",
		"10", "000", "0000", "0100", "000",
		0xc520,
		{
			{"SA[51:48]", 51, 48},
			{"SA[47:16]", 47, 16},
			{"Valid", 0, 0},
		}
	},
	{
		"LOREA_EL1",
		"LORegion End Address (EL1)",
		"10", "000", "0000", "0100", "001",
		0xc521,
		{
			{"EA[51:48]", 51, 48},
			{"EA[47:16]", 47, 16},
		}
	},
	{
		"LORN_EL1",
		"LORegion Number (EL1)",
		"10", "000", "0000", "0100", "010",
		0xc522,
		{
			{"Num", 7, 0},
		}
	},
	{
		"LORC_EL1",
		"LORegion Control (EL1)",
		"10", "000", "0000", "0100", "011",
		0xc523,
		{
			{"DS", 9, 2},
			{"EN", 0, 0},
		}
	},
	{
		"MPAMIDR_EL1",
		"MPAM ID Register (EL1)",
		"10", "000", "0000", "0100", "000",
		0xc524,
		{
			{"PMG_MAX", 39, 32},
			{"VPMR_MAX", 20, 18},
			{"HAS_HCR", 17, 17},
			{"PARTID_MAX", 15, 0},
		}
	},
	{
		"LORID_EL1",
		"LORegionID (EL1)",
		"10", "000", "0000", "0100", "011",
		0xc527,
		{
			{"LD", 23, 16},
			{"LR", 7, 0},
		}
	},
	{
		"MPAM1_EL1",
		"MPAM1 Register (EL1)",
		"10", "000", "0000", "0100", "000",
		0xc528,
		{
			{"MPAMEN", 63, 63},
			{"PMG_D", 47, 40},
			{"PMG_I", 39, 32},
			{"PARTID_D", 31, 16},
			{"PARTID_I", 15, 0},
		}
	},
	{
		"MPAM0_EL1",
		"MPAM0 Register (EL1)",
		"10", "000", "0000", "0100", "001",
		0xc529,
		{
			{"PMG_D", 47, 40},
			{"PMG_I", 39, 32},
			{"PARTID_D", 31, 16},
			{"PARTID_I", 15, 0},
		}
	},
	{
		"VBAR_EL1",
		"Vector Base Address Register (EL1)",
		"10", "000", "0100", "0000", "000",
		0xc600,
		{
			{"None", 63, 11},
		}
	},
	{
		"RVBAR_EL1",
		"Reset Vector Base Address Register (if EL2 and EL3 not implemented)",
		"10", "000", "0100", "0000", "001",
		0xc601,
		{
			{"None", 63, 0},
		}
	},
	{
		"RMR_EL1",
		"Reset Management Register (EL1)",
		"10", "000", "0100", "0000", "010",
		0xc602,
		{
			{"RR", 1, 1},
			{"AA64", 0, 0},
		}
	},
	{
		"ISR_EL1",
		"Interrupt Status Register",
		"10", "000", "0100", "0000", "000",
		0xc608,
		{
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
		}
	},
	{
		"DISR_EL1",
		"Deferred Interrupt Status Register",
		"10", "000", "0100", "0000", "001",
		0xc609,
		{
			{"A", 31, 31},
			{"IDS", 24, 24},
			{"AET", 12, 10},
			{"EA", 9, 9},
			{"DFSC", 5, 0},
		}
	},
	{
		"ICC_IAR0_EL1",
		"Interrupt Controller Interrupt Acknowledge Register 0",
		"10", "000", "0100", "0000", "000",
		0xc640,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_EOIR0_EL1",
		"Interrupt Controller End Of Interrupt Register 0",
		"10", "000", "0100", "0000", "001",
		0xc641,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_HPPIR0_EL1",
		"Interrupt Controller Highest Priority Pending Interrupt Register 0",
		"10", "000", "0100", "0000", "010",
		0xc642,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_BPR0_EL1",
		"Interrupt Controller Binary Point Register 0",
		"10", "000", "0100", "0000", "011",
		0xc643,
		{
			{"BinaryPoint", 2, 0},
		}
	},
	{
		"ICC_AP0R0_EL1",
		"Interrupt Controller Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "000",
		0xc644,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP0R1_EL1",
		"Interrupt Controller Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "010",
		0xc646,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP0R2_EL1",
		"Interrupt Controller Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "001",
		0xc645,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP0R3_EL1",
		"Interrupt Controller Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "011",
		0xc647,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP1R0_EL1",
		"Interrupt Controller Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "000",
		0xc648,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP1R1_EL1",
		"Interrupt Controller Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "010",
		0xc64a,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP1R2_EL1",
		"Interrupt Controller Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "001",
		0xc649,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_AP1R3_EL1",
		"Interrupt Controller Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "011",
		0xc64b,
		{
			{"IMPLEMENTATION DEFINED", 31, 0},
		}
	},
	{
		"ICC_DIR_EL1",
		"Interrupt Controller Deactivate Interrupt Register",
		"10", "000", "0100", "0000", "001",
		0xc659,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_RPR_EL1",
		"Interrupt Controller Running Priority Register",
		"10", "000", "0100", "0000", "011",
		0xc65b,
		{
			{"Priority", 7, 0},
		}
	},
	{
		"ICC_SGI1R_EL1",
		"Interrupt Controller Software Generated Interrupt Group 1 Register",
		"10", "000", "0100", "0000", "001",
		0xc65d,
		{
			{"Aff3", 55, 48},
			{"RS", 47, 44},
			{"IRM", 40, 40},
			{"Aff2", 39, 32},
			{"INTID", 27, 24},
			{"Aff1", 23, 16},
			{"TargetList", 15, 0},
		}
	},
	{
		"ICC_ASGI1R_EL1",
		"Interrupt Controller Alias Software Generated Interrupt Group 1 Register",
		"10", "000", "0100", "0000", "010",
		0xc65e,
		{
			{"Aff3", 55, 48},
			{"RS", 47, 44},
			{"IRM", 40, 40},
			{"Aff2", 39, 32},
			{"INTID", 27, 24},
			{"Aff1", 23, 16},
			{"TargetList", 15, 0},
		}
	},
	{
		"ICC_SGI0R_EL1",
		"Interrupt Controller Software Generated Interrupt Group 0 Register",
		"10", "000", "0100", "0000", "011",
		0xc65f,
		{
			{"Aff3", 55, 48},
			{"RS", 47, 44},
			{"IRM", 40, 40},
			{"Aff2", 39, 32},
			{"INTID", 27, 24},
			{"Aff1", 23, 16},
			{"TargetList", 15, 0},
		}
	},
	{
		"ICC_IAR1_EL1",
		"Interrupt Controller Interrupt Acknowledge Register 1",
		"10", "000", "0100", "0100", "000",
		0xc660,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_EOIR1_EL1",
		"Interrupt Controller End Of Interrupt Register 1",
		"10", "000", "0100", "0100", "001",
		0xc661,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_HPPIR1_EL1",
		"Interrupt Controller Highest Priority Pending Interrupt Register 1",
		"10", "000", "0100", "0100", "010",
		0xc662,
		{
			{"INTID", 23, 0},
		}
	},
	{
		"ICC_BPR1_EL1",
		"Interrupt Controller Binary Point Register 1",
		"10", "000", "0100", "0100", "011",
		0xc663,
		{
			{"BinaryPoint", 2, 0},
		}
	},
	{
		"ICC_CTLR_EL1",
		"Interrupt Controller Control Register (EL1)",
		"10", "000", "0100", "0100", "000",
		0xc664,
		{
			{"ExtRange", 19, 19},
			{"RSS", 18, 18},
			{"A3V", 15, 15},
			{"SEIS", 14, 14},
			{"IDbits", 13, 11},
			{"PRIbits", 10, 8},
			{"PMHE", 6, 6},
			{"EOImode", 1, 1},
			{"CBPR", 0, 0},
		}
	},
	{
		"ICC_SRE_EL1",
		"Interrupt Controller System Register Enable register (EL1)",
		"10", "000", "0100", "0100", "001",
		0xc665,
		{
			{"DIB", 2, 2},
			{"DFB", 1, 1},
			{"SRE", 0, 0},
		}
	},
	{
		"ICC_IGRPEN0_EL1",
		"Interrupt Controller Interrupt Group 0 Enable register",
		"10", "000", "0100", "0100", "010",
		0xc666,
		{
			{"Enable", 0, 0},
		}
	},
	{
		"ICC_IGRPEN1_EL1",
		"Interrupt Controller Interrupt Group 1 Enable register",
		"10", "000", "0100", "0100", "011",
		0xc667,
		{
			{"Enable", 0, 0},
		}
	},
	{
		"CONTEXTIDR_EL1",
		"Context ID Register (EL1)",
		"10", "000", "0100", "0000", "001",
		0xc681,
		{
			{"PROCID", 31, 0},
		}
	},
	{
		"TPIDR_EL1",
		"EL1 Software Thread ID Register",
		"10", "000", "0100", "0000", "000",
		0xc684,
		{
			{"None", 63, 0},
		}
	},
	{
		"SCXTNUM_EL1",
		"EL1 Read/Write Software Context Number",
		"10", "000", "0100", "0000", "011",
		0xc687,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTKCTL_EL1",
		"Counter-timer Kernel Control register",
		"10", "000", "0100", "0000", "000",
		0xc708,
		{
			{"EL0PTEN", 9, 9},
			{"EL0VTEN", 8, 8},
			{"EVNTI", 7, 4},
			{"EVNTDIR", 3, 3},
			{"EVNTEN", 2, 2},
			{"EL0VCTEN", 1, 1},
			{"EL0PCTEN", 0, 0},
		}
	},
	{
		"CCSIDR_EL1",
		"Current Cache Size ID Register",
		"10", "001", "0000", "0000", "000",
		0xc800,
		{
			{"NumSets", 55, 32},
			{"Associativity", 23, 3},
			{"LineSize", 2, 0},
		}
	},
	{
		"CLIDR_EL1",
		"Cache Level ID Register",
		"10", "001", "0000", "0000", "001",
		0xc801,
		{
			{"Ttype<n>", 46, 33},
			{"ICB", 32, 30},
			{"LoUU", 29, 27},
			{"LoC", 26, 24},
			{"LoUIS", 23, 21},
			{"Ctype<n>", 20, 0},
		}
	},
	{
		"CCSIDR2_EL1",
		"Current Cache Size ID Register 2",
		"10", "001", "0000", "0000", "010",
		0xc802,
		{
			{"NumSets", 23, 0},
		}
	},
	{
		"GMID_EL1",
		"Multiple tag transfer ID register",
		"10", "001", "0000", "0000", "000",
		0xc804,
		{
			{"BS", 3, 0},
		}
	},
	{
		"AIDR_EL1",
		"Auxiliary ID Register",
		"10", "001", "0000", "0000", "011",
		0xc807,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"CSSELR_EL1",
		"Cache Size Selection Register",
		"10", "010", "0000", "0000", "000",
		0xd000,
		{
			{"TnD", 4, 4},
			{"Level", 3, 1},
			{"InD", 0, 0},
		}
	},
	{
		"CTR_EL0",
		"Cache Type Register",
		"10", "011", "0000", "0000", "001",
		0xd801,
		{
			{"TminLine", 37, 32},
			{"DIC", 29, 29},
			{"IDC", 28, 28},
			{"CWG", 27, 24},
			{"ERG", 23, 20},
			{"DminLine", 19, 16},
			{"L1Ip", 15, 14},
			{"IminLine", 3, 0},
		}
	},
	{
		"DCZID_EL0",
		"Data Cache Zero ID register",
		"10", "011", "0000", "0000", "011",
		0xd807,
		{
			{"DZP", 4, 4},
			{"BS", 3, 0},
		}
	},
	{
		"RNDR",
		"Random Number",
		"10", "011", "0000", "0100", "000",
		0xd920,
		{
			{"RNDR", 63, 0},
		}
	},
	{
		"RNDRRS",
		"Reseeded Random Number",
		"10", "011", "0000", "0100", "001",
		0xd921,
		{
			{"RNDRRS", 63, 0},
		}
	},
	{
		"NZCV",
		"Condition Flags",
		"10", "011", "0100", "0000", "000",
		0xda10,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
		}
	},
	{
		"DAIF",
		"Interrupt Mask Bits",
		"10", "011", "0100", "0000", "001",
		0xda11,
		{
			{"D", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
		}
	},
	{
		"DIT",
		"Data Independent Timing",
		"10", "011", "0100", "0000", "001",
		0xda15,
		{
			{"DIT", 24, 24},
		}
	},
	{
		"SSBS",
		"Speculative Store Bypass Safe",
		"10", "011", "0100", "0000", "010",
		0xda16,
		{
			{"SSBS", 12, 12},
		}
	},
	{
		"TCO",
		"Tag Check Override",
		"10", "011", "0100", "0000", "011",
		0xda17,
		{
			{"TCO", 25, 25},
		}
	},
	{
		"FPCR",
		"Floating-point Control Register",
		"10", "011", "0100", "0100", "000",
		0xda20,
		{
			{"AHP", 26, 26},
			{"DN", 25, 25},
			{"FZ", 24, 24},
			{"RMode", 23, 22},
			{"Stride", 21, 20},
			{"FZ16", 19, 19},
			{"Len", 18, 16},
			{"IDE", 15, 15},
			{"IXE", 12, 12},
			{"UFE", 11, 11},
			{"OFE", 10, 10},
			{"DZE", 9, 9},
			{"IOE", 8, 8},
		}
	},
	{
		"FPSR",
		"Floating-point Status Register",
		"10", "011", "0100", "0100", "001",
		0xda21,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"QC", 27, 27},
			{"IDC", 7, 7},
			{"IXC", 4, 4},
			{"UFC", 3, 3},
			{"OFC", 2, 2},
			{"DZC", 1, 1},
			{"IOC", 0, 0},
		}
	},
	{
		"DSPSR_EL0",
		"Debug Saved Program Status Register",
		"10", "011", "0100", "0100", "000",
		0xda28,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"DIT", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"SS", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4]", 4, 4},
			{"M[3:0]", 3, 0},
		}
	},
	{
		"DLR_EL0",
		"Debug Link Register",
		"10", "011", "0100", "0100", "001",
		0xda29,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMCR_EL0",
		"Performance Monitors Control Register",
		"10", "011", "0000", "0100", "000",
		0xdce0,
		{
			{"IMP", 31, 24},
			{"IDCODE", 23, 16},
			{"N", 15, 11},
			{"LP", 7, 7},
			{"LC", 6, 6},
			{"DP", 5, 5},
			{"X", 4, 4},
			{"D", 3, 3},
			{"C", 2, 2},
			{"P", 1, 1},
			{"E", 0, 0},
		}
	},
	{
		"PMCNTENSET_EL0",
		"Performance Monitors Count Enable Set register",
		"10", "011", "0000", "0100", "001",
		0xdce1,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"PMCNTENCLR_EL0",
		"Performance Monitors Count Enable Clear register",
		"10", "011", "0000", "0100", "010",
		0xdce2,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"PMOVSCLR_EL0",
		"Performance Monitors Overflow Flag Status Clear Register",
		"10", "011", "0000", "0100", "011",
		0xdce3,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"PMSWINC_EL0",
		"Performance Monitors Software Increment register",
		"10", "011", "0000", "0100", "000",
		0xdce4,
		{
			{"P<n>", 30, 0},
		}
	},
	{
		"PMSELR_EL0",
		"Performance Monitors Event Counter Selection Register",
		"10", "011", "0000", "0100", "001",
		0xdce5,
		{
			{"SEL", 4, 0},
		}
	},
	{
		"PMCEID0_EL0",
		"Performance Monitors Common Event Identification register 0",
		"10", "011", "0000", "0100", "010",
		0xdce6,
		{
			{"IDhi<n>", 63, 32},
			{"ID<n>", 31, 0},
		}
	},
	{
		"PMCEID1_EL0",
		"Performance Monitors Common Event Identification register 1",
		"10", "011", "0000", "0100", "011",
		0xdce7,
		{
			{"IDhi<n>", 63, 32},
			{"ID<n>", 31, 0},
		}
	},
	{
		"PMCCNTR_EL0",
		"Performance Monitors Cycle Count Register",
		"10", "011", "0000", "0100", "000",
		0xdce8,
		{
			{"CCNT", 63, 0},
		}
	},
	{
		"PMXEVTYPER_EL0",
		"Performance Monitors Selected Event Type Register",
		"10", "011", "0000", "0100", "001",
		0xdce9,
		{
			{"Event type register or PMCCFILTR_EL0", 31, 0},
		}
	},
	{
		"PMXEVCNTR_EL0",
		"Performance Monitors Selected Event Count Register",
		"10", "011", "0000", "0100", "010",
		0xdcea,
		{
			{"PMEVCNTR<n>", 63, 0},
		}
	},
	{
		"PMUSERENR_EL0",
		"Performance Monitors User Enable Register",
		"10", "011", "0000", "0100", "000",
		0xdcf0,
		{
			{"ER", 3, 3},
			{"CR", 2, 2},
			{"SW", 1, 1},
			{"EN", 0, 0},
		}
	},
	{
		"PMOVSSET_EL0",
		"Performance Monitors Overflow Flag Status Set register",
		"10", "011", "0000", "0100", "011",
		0xdcf3,
		{
			{"C", 31, 31},
			{"P<n>", 30, 0},
		}
	},
	{
		"TPIDR_EL0",
		"EL0 Read/Write Software Thread ID Register",
		"10", "011", "0100", "0000", "010",
		0xde82,
		{
			{"None", 63, 0},
		}
	},
	{
		"TPIDRRO_EL0",
		"EL0 Read-Only Software Thread ID Register",
		"10", "011", "0100", "0000", "011",
		0xde83,
		{
			{"None", 63, 0},
		}
	},
	{
		"SCXTNUM_EL0",
		"EL0 Read/Write Software Context Number",
		"10", "011", "0100", "0000", "011",
		0xde87,
		{
			{"None", 63, 0},
		}
	},
	{
		"AMCR_EL0",
		"Activity Monitors Control Register",
		"10", "011", "0100", "0000", "000",
		0xde90,
		{
			{"HDBG", 10, 10},
			{"0", 9, 0},
		}
	},
	{
		"AMCFGR_EL0",
		"Activity Monitors Configuration Register",
		"10", "011", "0100", "0000", "001",
		0xde91,
		{
			{"NCG", 31, 28},
			{"HDBG", 24, 24},
			{"0", 23, 14},
			{"SIZE", 13, 8},
			{"N", 7, 0},
		}
	},
	{
		"AMCGCR_EL0",
		"Activity Monitors Counter Group Configuration Register",
		"10", "011", "0100", "0000", "010",
		0xde92,
		{
			{"CG1NC", 15, 8},
			{"CG0NC", 7, 0},
		}
	},
	{
		"AMUSERENR_EL0",
		"Activity Monitors User Enable Register",
		"10", "011", "0100", "0000", "011",
		0xde93,
		{
			{"0", 3, 1},
			{"EN", 0, 0},
		}
	},
	{
		"AMCNTENCLR0_EL0",
		"Activity Monitors Count Enable Clear Register 0",
		"10", "011", "0100", "0000", "000",
		0xde94,
		{
			{"P<n>", 31, 0},
		}
	},
	{
		"AMCNTENSET0_EL0",
		"Activity Monitors Count Enable Set Register 0",
		"10", "011", "0100", "0000", "001",
		0xde95,
		{
			{"P<n>", 31, 0},
		}
	},
	{
		"AMCNTENCLR1_EL0",
		"Activity Monitors Count Enable Clear Register 1",
		"10", "011", "0100", "0000", "000",
		0xde98,
		{
			{"P<n>", 31, 0},
		}
	},
	{
		"AMCNTENSET1_EL0",
		"Activity Monitors Count Enable Set Register 1",
		"10", "011", "0100", "0000", "001",
		0xde99,
		{
			{"P<n>", 31, 0},
		}
	},
	{
		"AMEVCNTR00_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdea0,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR01_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdea8,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR02_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdea4,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR03_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdeac,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR04_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdea2,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR05_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdeaa,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR06_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdea6,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR07_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdeae,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR08_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdea1,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR09_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdea9,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR010_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdea5,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR011_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdead,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR012_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdea3,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR013_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdeab,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR014_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdea7,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR015_EL0",
		"Activity Monitors Event Counter Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdeaf,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVTYPER00_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdeb0,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER01_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdeb8,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER02_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdeb4,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER03_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "000",
		0xdebc,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER04_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdeb2,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER05_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdeba,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER06_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdeb6,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER07_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "010",
		0xdebe,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER08_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdeb1,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER09_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdeb9,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER010_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdeb5,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER011_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "001",
		0xdebd,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER012_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdeb3,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER013_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdebb,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER014_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdeb7,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER015_EL0",
		"Activity Monitors Event Type Registers 0",
		"10", "011", "0100", "0100", "011",
		0xdebf,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVCNTR10_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdee0,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR11_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdee8,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR12_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdee4,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR13_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdeec,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR14_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdee2,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR15_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdeea,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR16_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdee6,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR17_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdeee,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR18_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdee1,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR19_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdee9,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR110_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdee5,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR111_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdeed,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR112_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdee3,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR113_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdeeb,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR114_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdee7,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVCNTR115_EL0",
		"Activity Monitors Event Counter Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdeef,
		{
			{"ACNT", 63, 0},
		}
	},
	{
		"AMEVTYPER10_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdef0,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER11_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdef8,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER12_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdef4,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER13_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "000",
		0xdefc,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER14_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdef2,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER15_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdefa,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER16_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdef6,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER17_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "010",
		0xdefe,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER18_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdef1,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER19_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdef9,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER110_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdef5,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER111_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "001",
		0xdefd,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER112_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdef3,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER113_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdefb,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER114_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdef7,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"AMEVTYPER115_EL0",
		"Activity Monitors Event Type Registers 1",
		"10", "011", "0100", "0100", "011",
		0xdeff,
		{
			{"0", 31, 25},
			{"evtCount", 15, 0},
		}
	},
	{
		"CNTFRQ_EL0",
		"Counter-timer Frequency register",
		"10", "011", "0100", "0000", "000",
		0xdf00,
		{
			{"None", 31, 0},
		}
	},
	{
		"CNTPCT_EL0",
		"Counter-timer Physical Count register",
		"10", "011", "0100", "0000", "001",
		0xdf01,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTVCT_EL0",
		"Counter-timer Virtual Count register",
		"10", "011", "0100", "0000", "010",
		0xdf02,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTP_TVAL_EL0",
		"Counter-timer Physical Timer TimerValue register",
		"10", "011", "0100", "0000", "000",
		0xdf10,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTP_CTL_EL0",
		"Counter-timer Physical Timer Control register",
		"10", "011", "0100", "0000", "001",
		0xdf11,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTP_CVAL_EL0",
		"Counter-timer Physical Timer CompareValue register",
		"10", "011", "0100", "0000", "010",
		0xdf12,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"CNTV_TVAL_EL0",
		"Counter-timer Virtual Timer TimerValue register",
		"10", "011", "0100", "0000", "000",
		0xdf18,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTV_CTL_EL0",
		"Counter-timer Virtual Timer Control register",
		"10", "011", "0100", "0000", "001",
		0xdf19,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTV_CVAL_EL0",
		"Counter-timer Virtual Timer CompareValue register",
		"10", "011", "0100", "0000", "010",
		0xdf1a,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"PMEVCNTR0_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf40,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR1_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf50,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR2_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf48,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR3_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf58,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR4_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf44,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR5_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf54,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR6_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf4c,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR7_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "000",
		0xdf5c,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR8_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf42,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR9_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf52,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR10_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf4a,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR11_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf5a,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR12_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf46,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR13_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf56,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR14_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf4e,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR15_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "010",
		0xdf5e,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR16_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf41,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR17_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf51,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR18_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf49,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR19_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf59,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR20_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf45,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR21_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf55,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR22_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf4d,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR23_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "001",
		0xdf5d,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR24_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf43,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR25_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf53,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR26_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf4b,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR27_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf5b,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR28_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf47,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR29_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf57,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR30_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf4f,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMEVCNTR31_EL0",
		"Performance Monitors Event Count Registers",
		"10", "011", "0100", "0000", "011",
		0xdf5f,
		{
			{"None", 63, 0},
		}
	},
	{
		"PMCCFILTR_EL0",
		"Performance Monitors Cycle Count Filter Register",
		"10", "011", "0100", "0100", "011",
		0xdf7f,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"SH", 24, 24},
		}
	},
	{
		"PMEVTYPER0_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf60,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER1_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf70,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER2_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf68,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER3_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf78,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER4_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf64,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER5_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf74,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER6_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf6c,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER7_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "000",
		0xdf7c,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER8_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf62,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER9_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf72,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER10_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf6a,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER11_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf7a,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER12_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf66,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER13_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf76,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER14_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf6e,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER15_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "010",
		0xdf7e,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER16_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf61,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER17_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf71,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER18_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf69,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER19_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf79,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER20_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf65,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER21_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf75,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER22_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf6d,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER23_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "001",
		0xdf7d,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER24_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf63,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER25_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf73,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER26_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf6b,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER27_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf7b,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER28_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf67,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER29_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf77,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER30_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf6f,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"PMEVTYPER31_EL0",
		"Performance Monitors Event Type Registers",
		"10", "011", "0100", "0100", "011",
		0xdf7f,
		{
			{"P", 31, 31},
			{"U", 30, 30},
			{"NSK", 29, 29},
			{"NSU", 28, 28},
			{"NSH", 27, 27},
			{"M", 26, 26},
			{"MT", 25, 25},
			{"SH", 24, 24},
			{"evtCount[15:10]", 15, 10},
			{"evtCount[9:0]", 9, 0},
		}
	},
	{
		"VPIDR_EL2",
		"Virtualization Processor ID Register",
		"10", "000", "0000", "0000", "000",
		0xe000,
		{
			{"Implementer", 31, 24},
			{"Variant", 23, 20},
			{"Architecture", 19, 16},
			{"PartNum", 15, 4},
			{"Revision", 3, 0},
		}
	},
	{
		"VMPIDR_EL2",
		"Virtualization Multiprocessor ID Register",
		"10", "000", "0000", "0000", "001",
		0xe005,
		{
			{"Aff3", 39, 32},
			{"U", 30, 30},
			{"MT", 24, 24},
			{"Aff2", 23, 16},
			{"Aff1", 15, 8},
			{"Aff0", 7, 0},
		}
	},
	{
		"SCTLR_EL2",
		"System Control Register (EL2)",
		"10", "000", "0000", "0000", "000",
		0xe080,
		{
			{"DSSBS", 44, 44},
			{"ATA", 43, 43},
			{"TCF", 41, 40},
			{"ITFSB", 37, 37},
			{"BT", 36, 36},
			{"EnIA", 31, 31},
			{"EnIB", 30, 30},
			{"EnDA", 27, 27},
			{"EE", 25, 25},
			{"EIS", 22, 22},
			{"IESB", 21, 21},
			{"WXN", 19, 19},
			{"EnDB", 13, 13},
			{"I", 12, 12},
			{"EOS", 11, 11},
			{"nAA", 6, 6},
			{"SA", 3, 3},
			{"C", 2, 2},
			{"A", 1, 1},
			{"M", 0, 0},
		}
	},
	{
		"ACTLR_EL2",
		"Auxiliary Control Register (EL2)",
		"10", "000", "0000", "0000", "001",
		0xe081,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"HCR_EL2",
		"Hypervisor Configuration Register",
		"10", "000", "0000", "0000", "000",
		0xe088,
		{
			{"TID5", 58, 58},
			{"DCT", 57, 57},
			{"ATA", 56, 56},
			{"TTLBOS", 55, 55},
			{"TTLBIS", 54, 54},
			{"EnSCXT", 53, 53},
			{"TOCU", 52, 52},
			{"TICAB", 50, 50},
			{"TID4", 49, 49},
			{"FIEN", 47, 47},
			{"FWB", 46, 46},
			{"NV2", 45, 45},
			{"AT", 44, 44},
			{"NV1", 43, 43},
			{"NV1", 43, 43},
			{"NV", 42, 42},
			{"NV", 42, 42},
			{"API", 41, 41},
			{"APK", 40, 40},
			{"MIOCNCE", 38, 38},
			{"TEA", 37, 37},
			{"TERR", 36, 36},
			{"TLOR", 35, 35},
			{"E2H", 34, 34},
			{"ID", 33, 33},
			{"CD", 32, 32},
			{"RW", 31, 31},
			{"TRVM", 30, 30},
			{"HCD", 29, 29},
			{"TDZ", 28, 28},
			{"TGE", 27, 27},
			{"TVM", 26, 26},
			{"TTLB", 25, 25},
			{"TTLB", 25, 25},
			{"TPU", 24, 24},
			{"TPCP", 23, 23},
			{"TPC", 23, 23},
			{"TSW", 22, 22},
			{"TACR", 21, 21},
			{"TIDCP", 20, 20},
			{"TSC", 19, 19},
			{"TID3", 18, 18},
			{"TID2", 17, 17},
			{"TID1", 16, 16},
			{"TID0", 15, 15},
			{"TWE", 14, 14},
			{"TWI", 13, 13},
			{"DC", 12, 12},
			{"BSU", 11, 10},
			{"FB", 9, 9},
			{"VSE", 8, 8},
			{"VI", 7, 7},
			{"VF", 6, 6},
			{"AMO", 5, 5},
			{"IMO", 4, 4},
			{"FMO", 3, 3},
			{"PTW", 2, 2},
			{"SWIO", 1, 1},
			{"VM", 0, 0},
		}
	},
	{
		"MDCR_EL2",
		"Monitor Debug Configuration Register (EL2)",
		"10", "000", "0000", "0000", "001",
		0xe089,
		{
			{"HLP", 26, 26},
			{"HCCD", 23, 23},
			{"TTRF", 19, 19},
			{"HPMD", 17, 17},
			{"TPMS", 14, 14},
			{"E2PB", 13, 12},
			{"TDRA", 11, 11},
			{"TDOSA", 10, 10},
			{"TDOSA", 10, 10},
			{"TDA", 9, 9},
			{"TDE", 8, 8},
			{"HPME", 7, 7},
			{"TPM", 6, 6},
			{"TPMCR", 5, 5},
			{"HPMN", 4, 0},
		}
	},
	{
		"CPTR_EL2",
		"Architectural Feature Trap Register (EL2)",
		"10", "000", "0000", "0000", "010",
		0xe08a,
		{
			{"TCPAC", 31, 31},
			{"TAM", 30, 30},
			{"TTA", 20, 20},
			{"TFP", 10, 10},
			{"TZ", 8, 8},
		}
	},
	{
		"HSTR_EL2",
		"Hypervisor System Trap Register",
		"10", "000", "0000", "0000", "011",
		0xe08b,
		{
			{"T<n>", 15, 0},
		}
	},
	{
		"HACR_EL2",
		"Hypervisor Auxiliary Control Register",
		"10", "000", "0000", "0000", "011",
		0xe08f,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"ZCR_EL2",
		"SVE Control Register for EL2",
		"10", "000", "0000", "0000", "000",
		0xe090,
		{
			{"0", 8, 4},
			{"LEN", 3, 0},
		}
	},
	{
		"TRFCR_EL2",
		"Trace Filter Control Register (EL2)",
		"10", "000", "0000", "0000", "001",
		0xe091,
		{
			{"TS", 6, 5},
			{"CX", 3, 3},
			{"E2TRE", 1, 1},
			{"E0HTRE", 0, 0},
		}
	},
	{
		"SDER32_EL2",
		"AArch32 Secure Debug Enable Register",
		"10", "000", "0000", "0000", "001",
		0xe099,
		{
			{"SUNIDEN", 1, 1},
			{"SUIDEN", 0, 0},
		}
	},
	{
		"TTBR0_EL2",
		"Translation Table Base Register 0 (EL2)",
		"10", "000", "0000", "0000", "000",
		0xe100,
		{
			{"ASID", 63, 48},
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"TTBR1_EL2",
		"Translation Table Base Register 1 (EL2)",
		"10", "000", "0000", "0000", "001",
		0xe101,
		{
			{"ASID", 63, 48},
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"TCR_EL2",
		"Translation Control Register (EL2)",
		"10", "000", "0000", "0000", "010",
		0xe102,
		{
			{"TCMA", 30, 30},
			{"TBID", 29, 29},
			{"HWU62", 28, 28},
			{"HWU61", 27, 27},
			{"HWU60", 26, 26},
			{"HWU59", 25, 25},
			{"HPD", 24, 24},
			{"HD", 22, 22},
			{"HA", 21, 21},
			{"TBI", 20, 20},
			{"PS", 18, 16},
			{"TG0", 15, 14},
			{"SH0", 13, 12},
			{"ORGN0", 11, 10},
			{"IRGN0", 9, 8},
			{"T0SZ", 5, 0},
		}
	},
	{
		"VTTBR_EL2",
		"Virtualization Translation Table Base Register",
		"10", "000", "0000", "0000", "000",
		0xe108,
		{
			{"VMID[15:8]", 63, 56},
			{"VMID[7:0]", 55, 48},
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"VTCR_EL2",
		"Virtualization Translation Control Register",
		"10", "000", "0000", "0000", "010",
		0xe10a,
		{
			{"NSA", 30, 30},
			{"NSW", 29, 29},
			{"HWU62", 28, 28},
			{"HWU61", 27, 27},
			{"HWU60", 26, 26},
			{"HWU59", 25, 25},
			{"HD", 22, 22},
			{"HA", 21, 21},
			{"VS", 19, 19},
			{"PS", 18, 16},
			{"TG0", 15, 14},
			{"SH0", 13, 12},
			{"ORGN0", 11, 10},
			{"IRGN0", 9, 8},
			{"SL0", 7, 6},
			{"SL0", 7, 6},
			{"T0SZ", 5, 0},
		}
	},
	{
		"VNCR_EL2",
		"Virtual Nested Control Register",
		"10", "000", "0000", "0000", "000",
		0xe110,
		{
			{"RESS", 63, 53},
			{"BADDR", 52, 12},
		}
	},
	{
		"VSTTBR_EL2",
		"Virtualization Secure Translation Table Base Register",
		"10", "000", "0000", "0100", "000",
		0xe130,
		{
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"VSTCR_EL2",
		"Virtualization Secure Translation Control Register",
		"10", "000", "0000", "0100", "010",
		0xe132,
		{
			{"SA", 30, 30},
			{"SW", 29, 29},
			{"TG0", 15, 14},
			{"SL0", 7, 6},
			{"SL0", 7, 6},
			{"T0SZ", 5, 0},
		}
	},
	{
		"DACR32_EL2",
		"Domain Access Control Register",
		"10", "000", "0000", "0000", "000",
		0xe180,
		{
			{"D<n>", 31, 0},
		}
	},
	{
		"SPSR_EL2",
		"Saved Program Status Register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe200,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"DIT", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"SS", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4]", 4, 4},
			{"M[3:0]", 3, 0},
		}
	},
	{
		"ELR_EL2",
		"Exception Link Register (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe201,
		{
			{"None", 63, 0},
		}
	},
	{
		"SP_EL1",
		"Stack Pointer (EL1)",
		"10", "000", "0100", "0000", "000",
		0xe208,
		{
			{"None", 63, 0},
		}
	},
	{
		"SPSR_irq",
		"Saved Program Status Register (IRQ mode)",
		"10", "000", "0100", "0000", "000",
		0xe218,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"J", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"DIT", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4:0]", 4, 0},
		}
	},
	{
		"SPSR_abt",
		"Saved Program Status Register (Abort mode)",
		"10", "000", "0100", "0000", "001",
		0xe219,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"J", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"DIT", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4:0]", 4, 0},
		}
	},
	{
		"SPSR_und",
		"Saved Program Status Register (Undefined mode)",
		"10", "000", "0100", "0000", "010",
		0xe21a,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"J", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"DIT", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4:0]", 4, 0},
		}
	},
	{
		"SPSR_fiq",
		"Saved Program Status Register (FIQ mode)",
		"10", "000", "0100", "0000", "011",
		0xe21b,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"J", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"DIT", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4:0]", 4, 0},
		}
	},
	{
		"IFSR32_EL2",
		"Instruction Fault Status Register (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe281,
		{
			{"FnV", 16, 16},
			{"ExT", 12, 12},
			{"FS[4]", 10, 10},
			{"LPAE", 9, 9},
			{"FS[3:0]", 3, 0},
		}
	},
	{
		"AFSR0_EL2",
		"Auxiliary Fault Status Register 0 (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe288,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"AFSR1_EL2",
		"Auxiliary Fault Status Register 1 (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe289,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"ESR_EL2",
		"Exception Syndrome Register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe290,
		{
			{"EC", 31, 26},
			{"IL", 25, 25},
			{"ISS", 24, 0},
		}
	},
	{
		"VSESR_EL2",
		"Virtual SError Exception Syndrome Register",
		"10", "000", "0100", "0000", "011",
		0xe293,
		{
			{"AET", 15, 14},
			{"ExT", 12, 12},
		}
	},
	{
		"FPEXC32_EL2",
		"Floating-Point Exception Control register",
		"10", "000", "0100", "0000", "000",
		0xe298,
		{
			{"EX", 31, 31},
			{"EN", 30, 30},
			{"DEX", 29, 29},
			{"FP2V", 28, 28},
			{"VV", 27, 27},
			{"TFV", 26, 26},
			{"VECITR", 10, 8},
			{"IDF", 7, 7},
			{"IXF", 4, 4},
			{"UFF", 3, 3},
			{"OFF", 2, 2},
			{"DZF", 1, 1},
			{"IOF", 0, 0},
		}
	},
	{
		"FAR_EL2",
		"Fault Address Register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe300,
		{
			{"None", 63, 0},
		}
	},
	{
		"HPFAR_EL2",
		"Hypervisor IPA Fault Address Register",
		"10", "000", "0100", "0000", "000",
		0xe304,
		{
			{"NS", 63, 63},
			{"FIPA[51:48]", 43, 40},
			{"FIPA[47:12]", 39, 4},
		}
	},
	{
		"TFSR_EL2",
		"Tag Fail Status Register (EL2).",
		"10", "000", "0100", "0100", "000",
		0xe328,
		{
			{"TF1", 1, 1},
			{"TF0", 0, 0},
		}
	},
	{
		"PMSCR_EL2",
		"Statistical Profiling Control Register (EL2)",
		"10", "000", "0000", "0000", "000",
		0xe4c8,
		{
			{"PCT", 6, 6},
			{"TS", 5, 5},
			{"PA", 4, 4},
			{"CX", 3, 3},
			{"E2SPE", 1, 1},
			{"E0HSPE", 0, 0},
		}
	},
	{
		"MAIR_EL2",
		"Memory Attribute Indirection Register (EL2)",
		"10", "000", "0000", "0000", "000",
		0xe510,
		{
			{"Attr<n>", 63, 0},
		}
	},
	{
		"AMAIR_EL2",
		"Auxiliary Memory Attribute Indirection Register (EL2)",
		"10", "000", "0000", "0000", "000",
		0xe518,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"MPAMHCR_EL2",
		"MPAM Hypervisor Control Register (EL2)",
		"10", "000", "0000", "0100", "000",
		0xe520,
		{
			{"TRAP_MPAMIDR_EL1", 31, 31},
			{"GSTAPP_PLK", 8, 8},
			{"EL1_VPMEN", 1, 1},
			{"EL0_VPMEN", 0, 0},
		}
	},
	{
		"MPAMVPMV_EL2",
		"MPAM Virtual Partition Mapping Valid Register",
		"10", "000", "0000", "0100", "001",
		0xe521,
		{
			{"VPM_V<m>", 31, 0},
		}
	},
	{
		"MPAM2_EL2",
		"MPAM2 Register (EL2)",
		"10", "000", "0000", "0100", "000",
		0xe528,
		{
			{"MPAMEN", 63, 63},
			{"TRAPMPAM0EL1", 49, 49},
			{"TRAPMPAM1EL1", 48, 48},
			{"PMG_D", 47, 40},
			{"PMG_I", 39, 32},
			{"PARTID_D", 31, 16},
			{"PARTID_I", 15, 0},
		}
	},
	{
		"MPAMVPM0_EL2",
		"MPAM Virtual PARTID Mapping Register 0",
		"10", "000", "0000", "0100", "000",
		0xe530,
		{
			{"PhyPARTID3", 63, 48},
			{"PhyPARTID2", 47, 32},
			{"PhyPARTID1", 31, 16},
			{"PhyPARTID0", 15, 0},
		}
	},
	{
		"MPAMVPM1_EL2",
		"MPAM Virtual PARTID Mapping Register 1",
		"10", "000", "0000", "0100", "001",
		0xe531,
		{
			{"PhyPARTID7", 63, 48},
			{"PhyPARTID6", 47, 32},
			{"PhyPARTID5", 31, 16},
			{"PhyPARTID4", 15, 0},
		}
	},
	{
		"MPAMVPM2_EL2",
		"MPAM Virtual PARTID Mapping Register 2",
		"10", "000", "0000", "0100", "010",
		0xe532,
		{
			{"PhyPARTID11", 63, 48},
			{"PhyPARTID10", 47, 32},
			{"PhyPARTID9", 31, 16},
			{"PhyPARTID8", 15, 0},
		}
	},
	{
		"MPAMVPM3_EL2",
		"MPAM Virtual PARTID Mapping Register 3",
		"10", "000", "0000", "0100", "011",
		0xe533,
		{
			{"PhyPARTID15", 63, 48},
			{"PhyPARTID14", 47, 32},
			{"PhyPARTID13", 31, 16},
			{"PhyPARTID12", 15, 0},
		}
	},
	{
		"MPAMVPM4_EL2",
		"MPAM Virtual PARTID Mapping Register 4",
		"10", "000", "0000", "0100", "000",
		0xe534,
		{
			{"PhyPARTID19", 63, 48},
			{"PhyPARTID18", 47, 32},
			{"PhyPARTID17", 31, 16},
			{"PhyPARTID16", 15, 0},
		}
	},
	{
		"MPAMVPM5_EL2",
		"MPAM Virtual PARTID Mapping Register 5",
		"10", "000", "0000", "0100", "001",
		0xe535,
		{
			{"PhyPARTID23", 63, 48},
			{"PhyPARTID22", 47, 32},
			{"PhyPARTID21", 31, 16},
			{"PhyPARTID20", 15, 0},
		}
	},
	{
		"MPAMVPM6_EL2",
		"MPAM Virtual PARTID Mapping Register 6",
		"10", "000", "0000", "0100", "010",
		0xe536,
		{
			{"PhyPARTID27", 63, 48},
			{"PhyPARTID26", 47, 32},
			{"PhyPARTID25", 31, 16},
			{"PhyPARTID24", 15, 0},
		}
	},
	{
		"MPAMVPM7_EL2",
		"MPAM Virtual PARTID Mapping Register 7",
		"10", "000", "0000", "0100", "011",
		0xe537,
		{
			{"PhyPARTID31", 63, 48},
			{"PhyPARTID30", 47, 32},
			{"PhyPARTID29", 31, 16},
			{"PhyPARTID28", 15, 0},
		}
	},
	{
		"VBAR_EL2",
		"Vector Base Address Register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe600,
		{
			{"None", 63, 11},
		}
	},
	{
		"RVBAR_EL2",
		"Reset Vector Base Address Register (if EL3 not implemented)",
		"10", "000", "0100", "0000", "001",
		0xe601,
		{
			{"None", 63, 0},
		}
	},
	{
		"RMR_EL2",
		"Reset Management Register (EL2)",
		"10", "000", "0100", "0000", "010",
		0xe602,
		{
			{"RR", 1, 1},
			{"AA64", 0, 0},
		}
	},
	{
		"VDISR_EL2",
		"Virtual Deferred Interrupt Status Register",
		"10", "000", "0100", "0000", "001",
		0xe609,
		{
			{"A", 31, 31},
			{"IDS", 24, 24},
			{"ISS", 23, 0},
		}
	},
	{
		"ICH_AP0R0_EL2",
		"Interrupt Controller Hyp Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "000",
		0xe640,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP0R1_EL2",
		"Interrupt Controller Hyp Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "010",
		0xe642,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP0R2_EL2",
		"Interrupt Controller Hyp Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "001",
		0xe641,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP0R3_EL2",
		"Interrupt Controller Hyp Active Priorities Group 0 Registers",
		"10", "000", "0100", "0000", "011",
		0xe643,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP1R0_EL2",
		"Interrupt Controller Hyp Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "000",
		0xe648,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP1R1_EL2",
		"Interrupt Controller Hyp Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "010",
		0xe64a,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP1R2_EL2",
		"Interrupt Controller Hyp Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "001",
		0xe649,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICH_AP1R3_EL2",
		"Interrupt Controller Hyp Active Priorities Group 1 Registers",
		"10", "000", "0100", "0000", "011",
		0xe64b,
		{
			{"P<x>", 31, 0},
		}
	},
	{
		"ICC_SRE_EL2",
		"Interrupt Controller System Register Enable register (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe64d,
		{
			{"Enable", 3, 3},
			{"DIB", 2, 2},
			{"DFB", 1, 1},
			{"SRE", 0, 0},
		}
	},
	{
		"ICH_HCR_EL2",
		"Interrupt Controller Hyp Control Register",
		"10", "000", "0100", "0000", "000",
		0xe658,
		{
			{"EOIcount", 31, 27},
			{"TDIR", 14, 14},
			{"TSEI", 13, 13},
			{"TALL1", 12, 12},
			{"TALL0", 11, 11},
			{"TC", 10, 10},
			{"VGrp1DIE", 7, 7},
			{"VGrp1EIE", 6, 6},
			{"VGrp0DIE", 5, 5},
			{"VGrp0EIE", 4, 4},
			{"NPIE", 3, 3},
			{"LRENPIE", 2, 2},
			{"UIE", 1, 1},
			{"En", 0, 0},
		}
	},
	{
		"ICH_VTR_EL2",
		"Interrupt Controller VGIC Type Register",
		"10", "000", "0100", "0000", "001",
		0xe659,
		{
			{"PRIbits", 31, 29},
			{"PREbits", 28, 26},
			{"IDbits", 25, 23},
			{"SEIS", 22, 22},
			{"A3V", 21, 21},
			{"nV4", 20, 20},
			{"TDS", 19, 19},
			{"ListRegs", 4, 0},
		}
	},
	{
		"ICH_MISR_EL2",
		"Interrupt Controller Maintenance Interrupt State Register",
		"10", "000", "0100", "0000", "010",
		0xe65a,
		{
			{"VGrp1D", 7, 7},
			{"VGrp1E", 6, 6},
			{"VGrp0D", 5, 5},
			{"VGrp0E", 4, 4},
			{"NP", 3, 3},
			{"LRENP", 2, 2},
			{"U", 1, 1},
			{"EOI", 0, 0},
		}
	},
	{
		"ICH_EISR_EL2",
		"Interrupt Controller End of Interrupt Status Register",
		"10", "000", "0100", "0000", "011",
		0xe65b,
		{
			{"Status<n>", 15, 0},
		}
	},
	{
		"ICH_ELRSR_EL2",
		"Interrupt Controller Empty List Register Status Register",
		"10", "000", "0100", "0000", "001",
		0xe65d,
		{
			{"Status<n>", 15, 0},
		}
	},
	{
		"ICH_VMCR_EL2",
		"Interrupt Controller Virtual Machine Control Register",
		"10", "000", "0100", "0000", "011",
		0xe65f,
		{
			{"VPMR", 31, 24},
			{"VBPR0", 23, 21},
			{"VBPR1", 20, 18},
			{"VEOIM", 9, 9},
			{"VCBPR", 4, 4},
			{"VFIQEn", 3, 3},
			{"VAckCtl", 2, 2},
			{"VENG1", 1, 1},
			{"VENG0", 0, 0},
		}
	},
	{
		"ICH_LR0_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "000",
		0xe660,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR1_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "000",
		0xe668,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR2_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "000",
		0xe664,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR3_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "000",
		0xe66c,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR4_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "010",
		0xe662,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR5_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "010",
		0xe66a,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR6_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "010",
		0xe666,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR7_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "010",
		0xe66e,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR8_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "001",
		0xe661,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR9_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "001",
		0xe669,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR10_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "001",
		0xe665,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR11_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "001",
		0xe66d,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR12_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "011",
		0xe663,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR13_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "011",
		0xe66b,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR14_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "011",
		0xe667,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"ICH_LR15_EL2",
		"Interrupt Controller List Registers",
		"10", "000", "0100", "0100", "011",
		0xe66f,
		{
			{"State", 63, 62},
			{"HW", 61, 61},
			{"Group", 60, 60},
			{"Priority", 55, 48},
			{"pINTID", 44, 32},
			{"vINTID", 31, 0},
		}
	},
	{
		"CONTEXTIDR_EL2",
		"Context ID Register (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe681,
		{
			{"PROCID", 31, 0},
		}
	},
	{
		"TPIDR_EL2",
		"EL2 Software Thread ID Register",
		"10", "000", "0100", "0000", "010",
		0xe682,
		{
			{"None", 63, 0},
		}
	},
	{
		"SCXTNUM_EL2",
		"EL2 Read/Write Software Context Number",
		"10", "000", "0100", "0000", "011",
		0xe687,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTVOFF_EL2",
		"Counter-timer Virtual Offset register",
		"10", "000", "0100", "0000", "011",
		0xe703,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTHCTL_EL2",
		"Counter-timer Hypervisor Control register",
		"10", "000", "0100", "0000", "000",
		0xe708,
		{
			{"EVNTI", 7, 4},
			{"EVNTDIR", 3, 3},
			{"EVNTEN", 2, 2},
			{"EL1PCEN", 1, 1},
			{"EL1PCTEN", 0, 0},
		}
	},
	{
		"CNTHP_TVAL_EL2",
		"Counter-timer Physical Timer TimerValue register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe710,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTHP_CTL_EL2",
		"Counter-timer Hypervisor Physical Timer Control register",
		"10", "000", "0100", "0000", "001",
		0xe711,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTHP_CVAL_EL2",
		"Counter-timer Physical Timer CompareValue register (EL2)",
		"10", "000", "0100", "0000", "010",
		0xe712,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"CNTHV_TVAL_EL2",
		"Counter-timer Virtual Timer TimerValue Register (EL2)",
		"10", "000", "0100", "0000", "000",
		0xe718,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTHV_CTL_EL2",
		"Counter-timer Virtual Timer Control register (EL2)",
		"10", "000", "0100", "0000", "001",
		0xe719,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTHV_CVAL_EL2",
		"Counter-timer Virtual Timer CompareValue register (EL2)",
		"10", "000", "0100", "0000", "010",
		0xe71a,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"CNTHVS_TVAL_EL2",
		"Counter-timer Secure Virtual Timer TimerValue register (EL2)",
		"10", "000", "0100", "0100", "000",
		0xe720,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTHVS_CTL_EL2",
		"Counter-timer Secure Virtual Timer Control register (EL2)",
		"10", "000", "0100", "0100", "001",
		0xe721,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTHVS_CVAL_EL2",
		"Counter-timer Secure Virtual Timer CompareValue register (EL2)",
		"10", "000", "0100", "0100", "010",
		0xe722,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"CNTHPS_TVAL_EL2",
		"Counter-timer Secure Physical Timer TimerValue register (EL2)",
		"10", "000", "0100", "0100", "000",
		0xe728,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTHPS_CTL_EL2",
		"Counter-timer Secure Physical Timer Control register (EL2)",
		"10", "000", "0100", "0100", "001",
		0xe729,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTHPS_CVAL_EL2",
		"Counter-timer Secure Physical Timer CompareValue register (EL2)",
		"10", "000", "0100", "0100", "010",
		0xe72a,
		{
			{"CompareValue", 63, 0},
		}
	},
	{
		"SCTLR_EL3",
		"System Control Register (EL3)",
		"10", "010", "0000", "0000", "000",
		0xf080,
		{
			{"DSSBS", 44, 44},
			{"ATA", 43, 43},
			{"TCF", 41, 40},
			{"ITFSB", 37, 37},
			{"BT", 36, 36},
			{"EnIA", 31, 31},
			{"EnIB", 30, 30},
			{"EnDA", 27, 27},
			{"EE", 25, 25},
			{"EIS", 22, 22},
			{"IESB", 21, 21},
			{"WXN", 19, 19},
			{"EnDB", 13, 13},
			{"I", 12, 12},
			{"EOS", 11, 11},
			{"nAA", 6, 6},
			{"SA", 3, 3},
			{"C", 2, 2},
			{"A", 1, 1},
			{"M", 0, 0},
		}
	},
	{
		"ACTLR_EL3",
		"Auxiliary Control Register (EL3)",
		"10", "010", "0000", "0000", "001",
		0xf081,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"SCR_EL3",
		"Secure Configuration Register",
		"10", "010", "0000", "0000", "000",
		0xf088,
		{
			{"ATA", 26, 26},
			{"EnSCXT", 25, 25},
			{"FIEN", 21, 21},
			{"NMEA", 20, 20},
			{"EASE", 19, 19},
			{"EEL2", 18, 18},
			{"API", 17, 17},
			{"API", 17, 17},
			{"APK", 16, 16},
			{"TERR", 15, 15},
			{"TLOR", 14, 14},
			{"TWE", 13, 13},
			{"TWI", 12, 12},
			{"ST", 11, 11},
			{"RW", 10, 10},
			{"SIF", 9, 9},
			{"SIF", 9, 9},
			{"HCE", 8, 8},
			{"SMD", 7, 7},
			{"EA", 3, 3},
			{"FIQ", 2, 2},
			{"IRQ", 1, 1},
			{"NS", 0, 0},
		}
	},
	{
		"SDER32_EL3",
		"AArch32 Secure Debug Enable Register",
		"10", "010", "0000", "0000", "001",
		0xf089,
		{
			{"SUNIDEN", 1, 1},
			{"SUIDEN", 0, 0},
		}
	},
	{
		"CPTR_EL3",
		"Architectural Feature Trap Register (EL3)",
		"10", "010", "0000", "0000", "010",
		0xf08a,
		{
			{"TCPAC", 31, 31},
			{"TAM", 30, 30},
			{"TTA", 20, 20},
			{"TFP", 10, 10},
			{"EZ", 8, 8},
		}
	},
	{
		"ZCR_EL3",
		"SVE Control Register for EL3",
		"10", "010", "0000", "0000", "000",
		0xf090,
		{
			{"0", 8, 4},
			{"LEN", 3, 0},
		}
	},
	{
		"MDCR_EL3",
		"Monitor Debug Configuration Register (EL3)",
		"10", "010", "0000", "0000", "001",
		0xf099,
		{
			{"SCCD", 23, 23},
			{"EPMAD", 21, 21},
			{"EPMAD", 21, 21},
			{"EDAD", 20, 20},
			{"EDAD", 20, 20},
			{"EDAD", 20, 20},
			{"TTRF", 19, 19},
			{"STE", 18, 18},
			{"SPME", 17, 17},
			{"SPME", 17, 17},
			{"SDD", 16, 16},
			{"SPD32", 15, 14},
			{"NSPB", 13, 12},
			{"TDOSA", 10, 10},
			{"TDOSA", 10, 10},
			{"TDA", 9, 9},
			{"TPM", 6, 6},
		}
	},
	{
		"TTBR0_EL3",
		"Translation Table Base Register 0 (EL3)",
		"10", "010", "0000", "0000", "000",
		0xf100,
		{
			{"BADDR", 47, 1},
			{"CnP", 0, 0},
		}
	},
	{
		"TCR_EL3",
		"Translation Control Register (EL3)",
		"10", "010", "0000", "0000", "010",
		0xf102,
		{
			{"TCMA", 30, 30},
			{"TBID", 29, 29},
			{"HWU62", 28, 28},
			{"HWU61", 27, 27},
			{"HWU60", 26, 26},
			{"HWU59", 25, 25},
			{"HPD", 24, 24},
			{"HD", 22, 22},
			{"HA", 21, 21},
			{"TBI", 20, 20},
			{"PS", 18, 16},
			{"TG0", 15, 14},
			{"SH0", 13, 12},
			{"ORGN0", 11, 10},
			{"IRGN0", 9, 8},
			{"T0SZ", 5, 0},
		}
	},
	{
		"SPSR_EL3",
		"Saved Program Status Register (EL3)",
		"10", "010", "0100", "0000", "000",
		0xf200,
		{
			{"N", 31, 31},
			{"Z", 30, 30},
			{"C", 29, 29},
			{"V", 28, 28},
			{"Q", 27, 27},
			{"IT[1:0]", 26, 25},
			{"DIT", 24, 24},
			{"SSBS", 23, 23},
			{"PAN", 22, 22},
			{"SS", 21, 21},
			{"IL", 20, 20},
			{"GE", 19, 16},
			{"IT[7:2]", 15, 10},
			{"E", 9, 9},
			{"A", 8, 8},
			{"I", 7, 7},
			{"F", 6, 6},
			{"T", 5, 5},
			{"M[4]", 4, 4},
			{"M[3:0]", 3, 0},
		}
	},
	{
		"ELR_EL3",
		"Exception Link Register (EL3)",
		"10", "010", "0100", "0000", "001",
		0xf201,
		{
			{"None", 63, 0},
		}
	},
	{
		"SP_EL2",
		"Stack Pointer (EL2)",
		"10", "010", "0100", "0000", "000",
		0xf208,
		{
			{"None", 63, 0},
		}
	},
	{
		"AFSR0_EL3",
		"Auxiliary Fault Status Register 0 (EL3)",
		"10", "010", "0100", "0000", "000",
		0xf288,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"AFSR1_EL3",
		"Auxiliary Fault Status Register 1 (EL3)",
		"10", "010", "0100", "0000", "001",
		0xf289,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"ESR_EL3",
		"Exception Syndrome Register (EL3)",
		"10", "010", "0100", "0000", "000",
		0xf290,
		{
			{"EC", 31, 26},
			{"IL", 25, 25},
			{"ISS", 24, 0},
		}
	},
	{
		"FAR_EL3",
		"Fault Address Register (EL3)",
		"10", "010", "0100", "0000", "000",
		0xf300,
		{
			{"None", 63, 0},
		}
	},
	{
		"TFSR_EL3",
		"Tag Fail Status Register (EL3).",
		"10", "010", "0100", "0100", "000",
		0xf330,
		{
			{"TF0", 0, 0},
		}
	},
	{
		"MAIR_EL3",
		"Memory Attribute Indirection Register (EL3)",
		"10", "010", "0000", "0000", "000",
		0xf510,
		{
			{"Attr<n>", 63, 0},
		}
	},
	{
		"AMAIR_EL3",
		"Auxiliary Memory Attribute Indirection Register (EL3)",
		"10", "010", "0000", "0000", "000",
		0xf518,
		{
			{"IMPLEMENTATION DEFINED", 63, 0},
		}
	},
	{
		"MPAM3_EL3",
		"MPAM3 Register (EL3)",
		"10", "010", "0000", "0100", "000",
		0xf528,
		{
			{"MPAMEN", 63, 63},
			{"TRAPLOWER", 62, 62},
			{"PMG_D", 47, 40},
			{"PMG_I", 39, 32},
			{"PARTID_D", 31, 16},
			{"PARTID_I", 15, 0},
		}
	},
	{
		"VBAR_EL3",
		"Vector Base Address Register (EL3)",
		"10", "010", "0100", "0000", "000",
		0xf600,
		{
			{"None", 63, 11},
		}
	},
	{
		"RVBAR_EL3",
		"Reset Vector Base Address Register (if EL3 implemented)",
		"10", "010", "0100", "0000", "001",
		0xf601,
		{
			{"None", 63, 0},
		}
	},
	{
		"RMR_EL3",
		"Reset Management Register (EL3)",
		"10", "010", "0100", "0000", "010",
		0xf602,
		{
			{"RR", 1, 1},
			{"AA64", 0, 0},
		}
	},
	{
		"ICC_CTLR_EL3",
		"Interrupt Controller Control Register (EL3)",
		"10", "010", "0100", "0100", "000",
		0xf664,
		{
			{"ExtRange", 19, 19},
			{"RSS", 18, 18},
			{"nDS", 17, 17},
			{"A3V", 15, 15},
			{"SEIS", 14, 14},
			{"IDbits", 13, 11},
			{"PRIbits", 10, 8},
			{"PMHE", 6, 6},
			{"RM", 5, 5},
			{"EOImode_EL1NS", 4, 4},
			{"EOImode_EL1S", 3, 3},
			{"EOImode_EL3", 2, 2},
			{"CBPR_EL1NS", 1, 1},
			{"CBPR_EL1S", 0, 0},
		}
	},
	{
		"ICC_SRE_EL3",
		"Interrupt Controller System Register Enable register (EL3)",
		"10", "010", "0100", "0100", "001",
		0xf665,
		{
			{"Enable", 3, 3},
			{"DIB", 2, 2},
			{"DFB", 1, 1},
			{"SRE", 0, 0},
		}
	},
	{
		"ICC_IGRPEN1_EL3",
		"Interrupt Controller Interrupt Group 1 Enable register (EL3)",
		"10", "010", "0100", "0100", "011",
		0xf667,
		{
			{"EnableGrp1S", 1, 1},
			{"EnableGrp1NS", 0, 0},
		}
	},
	{
		"TPIDR_EL3",
		"EL3 Software Thread ID Register",
		"10", "010", "0100", "0000", "010",
		0xf682,
		{
			{"None", 63, 0},
		}
	},
	{
		"SCXTNUM_EL3",
		"EL3 Read/Write Software Context Number",
		"10", "010", "0100", "0000", "011",
		0xf687,
		{
			{"None", 63, 0},
		}
	},
	{
		"CNTPS_TVAL_EL1",
		"Counter-timer Physical Secure Timer TimerValue register",
		"10", "011", "0100", "0000", "000",
		0xff10,
		{
			{"TimerValue", 31, 0},
		}
	},
	{
		"CNTPS_CTL_EL1",
		"Counter-timer Physical Secure Timer Control register",
		"10", "011", "0100", "0000", "001",
		0xff11,
		{
			{"ISTATUS", 2, 2},
			{"IMASK", 1, 1},
			{"ENABLE", 0, 0},
		}
	},
	{
		"CNTPS_CVAL_EL1",
		"Counter-timer Physical Secure Timer CompareValue register",
		"10", "011", "0100", "0000", "010",
		0xff12,
		{
			{"CompareValue", 63, 0},
		}
	},
};
