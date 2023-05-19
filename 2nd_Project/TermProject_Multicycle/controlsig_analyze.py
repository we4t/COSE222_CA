import openpyxl
workbook = openpyxl.Workbook()
worksheet = workbook.active

regdst_dict = {
    "00": "Rd(2nd Operand)",
    "01": "R15 - PC",
    "10": "R14 - LR",
    "xx": ""
}

regsrc_dict = {
    "00": "MDR",
    "01": "ALUOUT",
    "10": "ALUresult",
    "11": "B",
    "xx": ""
}

regBdst_dict = {
    "0": "Rm (third Operand)",
    "1": "Rd (second Operand)",
    "x": ""
}

ALUsrcA_dict = {
    "00": "PC",
    "01": "A",
    "10": "zero",
    "xx": ""
}

ALUsrcB_dict = {
    "00": "+4",
    "01": "+8",
    "10": "imm",
    "11": "B shift shmt",
    "xx": ""
}

immsrc_dict = {
    "00": "8_signext",
    "01": "12_zeroext",
    "10": "24_signext",
    "xx": ""
}
# 111 : xor | 011 : and | 010 : or
# 101 : adc | 100 : sbc | 001 : add | 000 : sub

ALUop_dict = {
    "0000": "and",
    "0001": "xor",
    "0010": "sub",
    "0100": "add",
    "0101": "adc",
    "0110": "sbc",
    "1010": "sub",
    "1100": "or",
    "1101": "add",
    "xxxx": ""
}


def get_instructions(inst):
    if inst == "B":
        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "00010110001001000100",
            "xxxxxxxxxxxxxxxxxxxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "BL":
        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "00011001001001000100",
            "00010101xxxxxxxx0xxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "STR":
        # Load/store immediate offset 	NZCV010PUBW0
        # Load/store register offset 	NZCV011PUBW0
        # ex_flags						111001011000
        # index							0123456789

        ex_flags = "111001011000"
        flags = ex_flags

        flag5 = "11" if flags[6] == "1" else "10"
        # decode immediate / register offest
        flag3 = "0100" if flags[8] == "1" else "0010"
        # decode whether offset is added from base (U == 1) or subtracted from base (U == 0)
        flag0 = "1"
        # decode the destination of regbdst

        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "0001010101" + flag5 + flag3 + "001" + flag0,
            "1000xxxxxxxxxxxx0xxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "LDR":
        # Load/store immediate offset 	NZCV010PUBW1
        # Load/store register offset 	NZCV011PUBW1
        # ex_flags						111001011001
        # index							0123456789

        ex_flags = "111001011001"
        flags = ex_flags

        flag5 = "11" if flags[6] == "1" else "10"
        # decode immediate / register offest
        flag3 = "0100" if flags[8] == "1" else "0010"
        # decode whether offset is added from base (U == 1) or subtracted from base (U == 0)
        flag0 = "0"
        # decode the destination of regbdst

        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "0001010101" + flag5 + flag3 + "001" + flag0,
            "0010xxxxxxxxxxxx0xxx",
            "00010000xxxxxxxx0xxx"
        ]

    elif inst == "CMP":
        # Data processing immediate shift 	: 	NZCV000opcdS
        # Data processing immediate 		: 	NZCV001opcdS
        # ex_flags = 							111000110101
        # index									0123456789

        ex_flags = "111000110101"
        flags = ex_flags

        flag5 = "10" if flags[6] == "1" else "11"
        # decode whether shift or not

        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "0001010101" + flag5 + "00101000",
            "xxxxxxxxxxxxxxxxxxxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "MOV":
        # Data processing immediate shift 	: 	NZCV000opcdS
        # Data processing immediate 		: 	NZCV001opcdS
        # ex_flags = 							111000111010
        # index									0123456789

        ex_flags = "111000111010"
        flags = ex_flags

        flag5 = "10" if flags[6] == "1" else "11"
        # decode whether shift or not
        flag0 = flags[11]
        # set flag, update NZCV

        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "0001010110" + flag5 + "0100" + flag0 + "000",
            "00010001xxxxxxxx0xxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "ALU":
        # Data processing immediate shift 	: 	NZCV000opcdS
        # Data processing immediate 		: 	NZCV001opcdS
        # ex_flags = 							111000001001
        # index									0123456789

        ex_flags = "111000001001"
        flags = ex_flags

        flag5 = "10" if flags[6] == "1" else "11"
        # decode whether shift or not
        flags_str = flags[7:]
        # opcode + NZCV update flag

        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "0001010101" + flag5 + flags_str + "000",
            "00010001xxxxxxxx0xxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]

    elif inst == "Recovery":
        return [
            "01110110000101000xxx",
            "0000xxxx000000100xxx",
            "00010101xxxxxxxx0xxx",
            "xxxxxxxxxxxxxxxxxxxx",
            "xxxxxxxxxxxxxxxxxxxx"
        ]


def print_signals(instruction):
    s = get_instructions(instruction)
    tmparr = [instruction, "Mwrite", "IRwrite", "Mread", "regwrite", "regdst",
              "regsrc", "ALUsrcA", "ALUsrcB", "ALUop", "NZCVwrite", "immsrc", "regbdst"]
    worksheet.append(tmparr)
    for step in range(len(s)):
        arr = []
        arr.append(step)
        Mwrite = s[step][0]
        arr.append(Mwrite)
        IRwrite = s[step][1]
        arr.append(IRwrite)
        Mread = s[step][2]
        arr.append(Mread)
        regwrite = s[step][3]
        arr.append(regwrite)
        regdst = s[step][4:6]
        arr.append(regdst + " " + regdst_dict[regdst])
        regsrc = s[step][6:8]
        arr.append(regsrc + " " + regsrc_dict[(regsrc)])
        ALUsrcA = s[step][8:10]
        arr.append(ALUsrcA + " " + ALUsrcA_dict[(ALUsrcA)])
        ALUsrcB = s[step][10:12]
        arr.append(ALUsrcB + " " + ALUsrcB_dict[(ALUsrcB)])
        ALUop = s[step][12:16]
        arr.append(ALUop + " " + ALUop_dict[(ALUop)])
        NZCVwrite = s[step][16]
        arr.append(NZCVwrite)
        immsrc = s[step][17:19]
        arr.append(immsrc + " " + immsrc_dict[(immsrc)])
        regbdst = s[step][19]
        arr.append(regbdst + " " + regBdst_dict[(regbdst)])

        print(f"Step {arr[0] + 1}:")
        print(f"Mwrite: {arr[1]}")
        print(f"IRwrite: {arr[2]}")
        print(f"Mread: {arr[3]}")
        print(f"regwrite: {arr[4]}")
        print(f"regdst: {arr[5]}")
        print(f"regsrc: {arr[6]}")
        print(f"ALUsrcA: {arr[7]}")
        print(f"ALUsrcB: {arr[8]}")
        print(f"ALUop: {arr[9]}")
        print(f"NZCVwrite: {arr[10]}")
        print(f"immsrc: {arr[11]}")
        print(f"regbdst: {arr[12]}")
        print()
        worksheet.append(arr)


# Example usage
instructions_arr = ["B", "BL", "STR", "LDR", "CMP", "MOV", "ALU", "Recovery"]

for instruction in instructions_arr:
    print(f"Instruction: {instruction}")
    print_signals(instruction)
    print()
    worksheet.append([])

workbook.save('controlsig_analyze.xlsx')
