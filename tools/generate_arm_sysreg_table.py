#!/usr/bin/env python

# Copyright (C) 2019 by Mete Balci
# metebalci@gmail.com
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Usage: run generate_arm_sysreg_table.py inside the unzipped
#  Arm SysReg xml folder:
# ./tools/generate_arm_sysreg_table <folder> > \
#        src/target/aarch64_system_registers.h

import xml.etree.ElementTree as ET
from sys import argv, exit


def remove_0b(entry):
    entry = entry.text
    if entry[:2] != '0b':
        raise Exception
    return entry[2:]


def fix_ns(bitstr):
    s = bitstr.find('[')
    if s >= 0:
        e = bitstr.find(']')
        if e >= 0:
            # omitting [n:
            grp = bitstr[s+3:e]
            parts = grp.split(':')
            if len(parts) == 1:
                r = 'n'
            elif len(parts) == 2:
                r = 'n' * (int(parts[0]) - int(parts[1]) + 1)
            else:
                raise Exception
            return fix_ns('%s%s%s' % (bitstr[:s], r, bitstr[e+1:]))
        else:
            raise Exception
    else:
        return bitstr


def parse_encoding(regname, op0, op1, crn, crm, op2):
    op0 = remove_0b(row[0])
    op1 = remove_0b(row[1])
    crn = remove_0b(row[2])
    crm = remove_0b(row[3])
    op2 = remove_0b(row[4])
    bitstr = op0 + op1 + crn + crm + op2
    bitstr = fix_ns(bitstr)
    npos = []
    for i in range(0, len(bitstr)):
        if bitstr[i] == 'n':
            npos.append(i)
    if len(npos) > 0:
        results = []
        for i in range(0, 1 << len(npos)):
            cbitstr = list(bitstr)
            for k in range(0, len(npos)):
                mask = (1 << k)
                if (i & mask) == 0:
                    cbitstr[npos[k]] = '0'
                else:
                    cbitstr[npos[k]] = '1'
            name = regname.replace('<n>', str(i))
            results.append((name, int(''.join(cbitstr), 2)))
        return results
    else:
        return [(regname, int(bitstr, 2))]


def encoding_to_fields(encoding):
    return (
        format((encoding >> 14) & 0x2, '02b'),
        format((encoding >> 11) & 0x3, '03b'),
        format((encoding >> 7) & 0x4, '04b'),
        format((encoding >> 3) & 0x4, '04b'),
        format(encoding & 0x3, '03b'))


def get_reg_info(regxml):
    ret = {}
    tree = ET.parse(regxml)
    register_page = tree.getroot()
    register = register_page.find('registers').find('register')
    ret['short_name'] = register.find('reg_short_name').text.strip()
    ret['long_name'] = register.find('reg_long_name').text.strip()
    pp = register.find('reg_purpose').find('purpose_text').find('para')
    if pp.text is not None:
        purpose = pp.text
    else:
        purpose = ''
    for c in pp:
        purpose = '%s%s' % (purpose, c.text)
        if c.tail is not None:
            purpose = '%s%s' % (purpose, c.tail)
    if pp.tail is not None:
        purpose = '%s%s' % (purpose, pp.tail)
    purpose = purpose.replace('\"', '').strip()
    ret['purpose'] = purpose
    ret['group'] = register.find('reg_groups').find('reg_group').text
    ret['fields'] = []
    reg_fieldsets = register.find('reg_fieldsets')
    if reg_fieldsets is None:
        return ret
    fields = reg_fieldsets.find('fields')
    if fields is None:
        return ret
    for field in fields.findall('field'):
        rwtype = field.attrib.get('rwtype', '')
        if rwtype == 'RES0':
            continue
        if rwtype == 'RES1':
            continue
        name = field.find('field_name')
        if name is not None:
            name = name.text.strip()
        msb = int(field.find('field_msb').text)
        lsb = int(field.find('field_lsb').text)
        pp = field.find('field_description').find('para')
        if pp.text is not None:
            desc = pp.text
        else:
            desc = ''
        for c in pp:
            desc = '%s%s' % (desc, c.text)
            if c.tail is not None:
                desc = '%s%s' % (desc, c.tail)
        if pp.tail is not None:
            desc = '%s%s' % (desc, pp.tail)
        desc = desc.replace('\"', '').strip()
        ret['fields'].append((name, msb, lsb, desc))
    return ret


if __name__ == '__main__':
    sectiongroup_groupname = 'AArch64'
    section_type = "MRS/MSR"
    regnames = []
    regrows = []
    regencodings = []
    reginfos = []
    # this file lists all system registers and links each to detail xml
    tree = ET.parse('enc_index.xml')
    sysregindex = tree.getroot()
    for sectiongroup in sysregindex.findall('sectiongroup'):
        if sectiongroup.get('groupname', None) == sectiongroup_groupname:
            for section in sectiongroup.findall('section'):
                if section.get('type', None) == section_type:
                    tbody = section.find('tbody')
                    for row in tbody.findall('row'):
                        reghref = row[5][0].get('href')
                        info = get_reg_info(reghref)
                        regname = row[5][0].text.strip()
                        entries = parse_encoding(regname,
                                                 row[0],
                                                 row[1],
                                                 row[2],
                                                 row[3],
                                                 row[4])
                        for (name, encoding) in entries:
                            regnames.append(name)
                            regencodings.append(encoding)
                            reginfos.append(info)
                            regrows.append(row)

    print("static const struct aarch64_system_register aarch64_system_registers[] = {")
    for i in range(0, len(regnames)):
        print("\t{")
        reginfo = reginfos[i]
        print("\t\t\"%s\"," % regnames[i])
        print("\t\t\"%s\"," % reginfo['long_name'])
        op0, op1, crN, crM, op2 = encoding_to_fields(regencodings[i])
        print("\t\t\"%s\", \"%s\", \"%s\", \"%s\", \"%s\"," %
              (op0, op1, crN, crM, op2))
        print("\t\t0x%x," % regencodings[i])
        print("\t\t{")
        for i in range(0, len(reginfo['fields'])):
            field = reginfo['fields'][i]
            print("\t\t\t{\"%s\", %u, %u}%s" % (field[0],
                                                field[1],
                                                field[2],
                                                ',' if i < 63 else ''))
        print("\t\t}")
        print("\t},")
    print("};")
