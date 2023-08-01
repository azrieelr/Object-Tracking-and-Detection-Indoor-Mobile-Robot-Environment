import sympy as sp

from coordinates.factory import createDCM_Body2Navigation

if __name__ == '__main__':
    cbn = createDCM_Body2Navigation()
    sp.pprint(cbn.dcm)

    cnb = cbn.T
    sp.pprint(cnb.dcm)