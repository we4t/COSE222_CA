f = open("insts_data.txt", "r")
arr = []

bin_str__dict = {
    '0': '0000',
    '1': '0001',
    '2': '0010',
    '3': '0011',
    '4': '0100',
    '5': '0101',
    '6': '0110',
    '7': '0111',
    '8': '1000',
    '9': '1001',
    'A': '1010',
    'B': '1011',
    'C': '1100',
    'D': '1101',
    'E': '1110',
    'F': '1111'
}

str_bin_dict = {value: key for key, value in bin_str__dict.items()}

opcode_dict = {
    '0000': 'AND Logical AND\nRd := Rn AND shifter_operand',
    '0001': 'EOR Logical Exclusive OR\nRd := Rn EOR shifter_operand',
    '0010': 'SUB Subtract\nRd := Rn - shifter_operand',
    '0011': 'RSB Reverse Subtract\nRd := shifter_operand - Rn',
    '0100': 'ADD Add\nRd := Rn + shifter_operand\n',
    '0101': 'ADC Add with Carry\nRd := Rn + shifter_operand + Carry Flag',
    '0110': 'SBC Subtract with Carry\nRd := Rn - shifter_operand - NOT(Carry Flag)',
    '0111': 'RSC Reverse Subtract with Carry Test\nRd := shifter_operand - Rn - NOT(Carry Flag)',
    '1000': 'TST Test\nUpdate flags after Rn AND shifter_operand',
    '1001': 'TEQ Test Equivalence\nUpdate flags after Rn EOR shifter_operand',
    '1010': 'CMP Compare\nUpdate flags after Rn - shifter_operand',
    '1011': 'CMN Compare Negated\nUpdate flags after Rn + shifter_operand',
    '1100': 'ORR Logical (inclusive) OR\nRd := Rn OR shifter_operand',
    '1101': 'MOV Move\nRd := shifter_operand (no first operand)',
    '1110': 'BIC Bit Clear\nRd := Rn AND NOT(shifter_operand)',
    '1111': 'MVN Move Not\nRd := NOT shifter_operand (no first operand)'
}

# temp
pubwl_ch_dict = {}
opcode_ch_dict = {}


def evaluate_binary(bin_str):
    """
    Evaluate the binary format string, considering the MSB with sign.
    :param bin_str: A binary format string to evaluate.
    :return: The decimal value of the binary string.
    """
    if bin_str[0] == '1':  # negative number
        return -(int(''.join(['1' if x == '0' else '0' for x in bin_str]), 2) + 1)
    else:  # positive number
        return int(bin_str, 2)


def evaluate_unsigned_binary(bin_str):
    return int(bin_str, 2)


for i in f:
    tmp = i.split(' ')
    if len(tmp) > 2:
        arr.append(tmp[5][:-2])

ins_num = 0

for hexmsg in arr:
    bin_str_ = ""
    ins_info = ""

    for ch in hexmsg:
        bin_str_ = bin_str_ + bin_str__dict[ch]

    cond = bin_str_[0:4]
    funct3 = bin_str_[4:7]

    if bin_str_[4:7] == "001":
        opcode = bin_str_[7:11]
        s = bin_str_[11]
        Rn = bin_str_[12:16]
        Rd = bin_str_[16:20]
        rotate = bin_str_[20:24]
        imm8 = bin_str_[24:]
        bin_str_ = "{0} {1} {2} {3} {4} {5} {6} {7}".format(
            cond, funct3, opcode, s, Rn, Rd, rotate, imm8)
        ins_info += "cond : always\n"
        ins_info += "funct3 : Data processing immediate\n"
        ins_info += "opcode : {0}\n".format(opcode_dict[opcode])
        opcode_ch_dict[opcode_dict[opcode]] = 0
        ins_info += "s: {0}\n".format(s)
        ins_info += "Rn: {0}\n".format(str_bin_dict[Rn])
        ins_info += "Rd: {0}\n".format(str_bin_dict[Rd])
        ins_info += "rotate: {0}\n".format(str_bin_dict[rotate])
        ins_info += "imm8: {0}\n".format(evaluate_binary(imm8))

    elif bin_str_[4:7] == "010":
        pubwl = bin_str_[7:12]
        Rn = bin_str_[12:16]
        Rd = bin_str_[16:20]
        imm12 = bin_str_[20:]
        bin_str_ = "{0} {1} {2} {3} {4} {5}".format(
            cond, funct3, pubwl, Rn, Rd, imm12)
        ins_info += "cond: always\n"
        ins_info += "funct3: Load/store immediate offset\n"
        ins_info += "pubwl: {0}\n".format(pubwl)
        if pubwl[-1] == '1':
            ins_info += "pubwl: Load Instruction\n"
        else:
            ins_info += "pubwl: Store Instruction\n"
        pubwl_ch_dict[pubwl] = 0
        ins_info += "Rn: {0}\n".format(str_bin_dict[Rn])
        ins_info += "Rd: {0}\n".format(str_bin_dict[Rd])
        ins_info += "imm12: {0}\n".format(evaluate_unsigned_binary(imm12))

    else:
        l = bin_str_[7]
        imm24 = bin_str_[8:]
        bin_str_ = "{0} {1} {2} {3}".format(cond, funct3, l, imm24)
        if cond == "0000":
            ins_info += "cond: EQ, z == 0\n"
        else:
            ins_info += "cond: always\n"
        ins_info += "funct3: branch and branch with link\n"
        ins_info += "l: {0}\n".format(l)
        ins_info += "imm24: {0}\n".format(evaluate_binary(imm24))
        ins_info += "next_pc: %X\n" % (ins_num + 2 + evaluate_binary(imm24))

    print("%03X : %s" % (ins_num, bin_str_))
    print(ins_info)

    ins_num = ins_num + 1
