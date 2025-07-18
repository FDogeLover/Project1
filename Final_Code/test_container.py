class Containers():
    dict_qrcode = {'https://5v60oq.58u.cn/a/RlYaaoM/': 1, 'https://5v60oq.58u.cn/a/bDNqqqx/': 2,
                   'https://5v60oq.58u.cn/a/RMkwwwj/': 3, 'https://5v60oq.58u.cn/a/Ay4eeeX/': 4,
                   'https://5v60oq.58u.cn/a/O8eDDDV/': 5, 'https://5v60oq.58u.cn/a/O9Dmmmy/': 6,
                   'https://5v60oq.58u.cn/a/OdB888K/': 7, 'https://5v60oq.58u.cn/a/AGwdddv/': 8,
                   'https://5v60oq.58u.cn/a/AQp7778/': 9, 'https://5v60oq.58u.cn/a/R4e222J/': 10,
                   'https://5v60oq.58u.cn/a/AV8YYYN/': 11, 'https://5v60oq.58u.cn/a/AxqWWWQ/': 12,
                   'https://5v60oq.58u.cn/a/RnJwwwB/': 13, 'https://5v60oq.58u.cn/a/Ag2999E/': 14,
                   'https://5v60oq.58u.cn/a/bKLaaaw/': 15, 'https://5v60oq.58u.cn/a/OExvvvX/': 16,
                   'https://5v60oq.58u.cn/a/O71eeeG/': 17, 'https://5v60oq.58u.cn/a/OzYDDDz/': 18,
                   'https://5v60oq.58u.cn/a/RlYaaaM/': 19, 'https://5v60oq.58u.cn/a/AB0PPPB/': 20,
                   'https://5v60oq.58u.cn/a/RYvDDQX/': 21, 'https://5v60oq.58u.cn/a/be2qqE3/': 22,
                   'https://5v60oq.58u.cn/a/RmYQQEk/': 23, 'https://5v60oq.58u.cn/a/b1YzzL7/': 24}

    list_key_str = ['https://5v60oq.58u.cn/a/RlYaaoM/', 'https://5v60oq.58u.cn/a/bDNqqqx/',
                    'https://5v60oq.58u.cn/a/RMkwwwj/', 'https://5v60oq.58u.cn/a/Ay4eeeX/',
                    'https://5v60oq.58u.cn/a/O8eDDDV/', 'https://5v60oq.58u.cn/a/O9Dmmmy/',
                    'https://5v60oq.58u.cn/a/OdB888K/', 'https://5v60oq.58u.cn/a/AGwdddv/',
                    'https://5v60oq.58u.cn/a/AQp7778/', 'https://5v60oq.58u.cn/a/R4e222J/',
                    'https://5v60oq.58u.cn/a/AV8YYYN/', 'https://5v60oq.58u.cn/a/AxqWWWQ/',
                    'https://5v60oq.58u.cn/a/RnJwwwB/', 'https://5v60oq.58u.cn/a/Ag2999E/',
                    'https://5v60oq.58u.cn/a/bKLaaaw/', 'https://5v60oq.58u.cn/a/OExvvvX/',
                    'https://5v60oq.58u.cn/a/O71eeeG/', 'https://5v60oq.58u.cn/a/OzYDDDz/',
                    'https://5v60oq.58u.cn/a/RlYaaaM/', 'https://5v60oq.58u.cn/a/AB0PPPB/',
                    'https://5v60oq.58u.cn/a/RYvDDQX/', 'https://5v60oq.58u.cn/a/be2qqE3/',
                    'https://5v60oq.58u.cn/a/RmYQQEk/', 'https://5v60oq.58u.cn/a/b1YzzL7/']

    # dict_route_index_pos = {3: 'A3', 5: 'A6', 6: 'A5', 8: 'A2', 9: 'A1', 11: 'A4',
    #                         16: ('C1', 'B3'), 18: ('C4', 'B6'), 19: ('C5', 'B5'), 21: ('C2', 'B2'), 22: ('C3', 'B1'), 24: ('C6', 'B4'),
    #                         29: 'D1', 31: 'D4', 32: 'D5', 34: 'D2', 35: 'D3', 37: 'D6'}
    dict_route_index_pos = {3: 'A3', 5: 'A6', 6: 'A5', 8: 'A2', 9: 'A1', 11: 'A4',
                            14: ('C4', 'B6'), 15: ('C5', 'B5'), 16: ('C6', 'B4'), 18: ('C3', 'B1'), 19: ('C2', 'B2'),
                            20: ('C1', 'B3'),
                            23: 'D3', 24: 'D2', 25: 'D1', 27: 'D4', 28: 'D5', 29: 'D6'}

    # 前往A3
    router_A3 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (-0.25, 0.75, 1.25, 3, 0, 0, 0),  #
                 (-0.25, 2.75, 1.25, 8, 0, 0, 0),
                 (3.5, 2.75, 1.25, 14, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]
    # 前往A6
    router_A6 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (-0.25, 0.75, 0.85, 3, 0, 0, 0),  #
                 (-0.25, 2.75, 0.85, 9, 0, 0, 0),
                 (3.5, 2.75, 0.85, 14, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]
    # 前往A5
    router_A5 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (-0.25, 1.25, 0.85, 5, 0, 0, 0),  #
                 (-0.25, 2.75, 0.85, 8, 0, 0, 0),
                 (3.5, 2.75, 0.85, 14, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]
    # 前往A2
    router_A2 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (-0.25, 1.25, 1.25, 5, 0, 0, 0),  #
                 (-0.25, 2.75, 1.25, 9, 0, 0, 0),
                 (3.5, 2.75, 1.25, 14, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]
    # 前往A1
    router_A1 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (-0.25, 1.75, 1.25, 8, 0, 0, 0),  #
                  (-0.25, 2.75, 1.25, 7, 0, 0, 0),
                 (3.5, 2.75, 1.25, 14, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]
    # 前往A4
    router_A4 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (-0.25, 1.75, 0.85, 8, 0, 0, 0),  #
                 (-0.25, 2.75, 0.85, 7, 0, 0, 0),
                 (3.5, 2.75, 0.85, 14, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]
    # B6 C4
    router_B6 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (1.75, -0.3, 0.85, 4, 0, 0, 0),
                 (1.75, 1.75, 0.85, 8, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 5, 0, 0, 0),
                 (3.5, 2.75, 0.85, 8, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    router_C4 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (1.75, -0.3, 0.85, 4, 0, 0, 0),
                 (1.75, 1.75, 0.85, 8, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 5, 0, 0, 0),
                 (3.5, 2.75, 0.85, 8, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    # B5 C5
    router_B5 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (1.75, -0.3, 0.85, 4, 0, 0, 0),
                 (1.75, 1.25, 0.85, 6, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    router_C5 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (1.75, -0.3, 0.85, 4, 0, 0, 0),
                 (1.75, 1.25, 0.85, 6, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    # B4 C6
    router_B4 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (1.75, -0.3, 0.85, 4, 0, 0, 0),
                 (1.75, 0.75, 0.85, 5, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 8, 0, 0, 0),
                 (3.5, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    router_C6 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.25, 0.85, 3, 0, 0, 0),
                 (1.75, -0.25, 0.85, 4, 0, 0, 0),
                 (1.75, 0.75, 0.85, 5, 0, 0, 0),  #
                 (1.75, 2.75, 0.85, 8, 0, 0, 0),
                 (3.5, 2.75, 0.85, 6, 0, 0, 0),
                 (3.5, 2.5, 0.85, 4, 0, 0, 0)]

    # B1 C3
    router_B1 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 0.75, 1.25, 5, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 8, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    router_C3 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 0.75, 1.25, 5, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 8, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    # B2 C2
    router_B2 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 1.25, 1.25, 6, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    router_C2 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 1.25, 1.25, 6, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    # B3 C1
    router_B3 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 1.75, 1.25, 8, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    router_C1 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.25, 1.25, 3, 0, 0, 0),
                 (1.75, -0.25, 1.25, 4, 0, 0, 0),
                 (1.75, 1.75, 1.25, 8, 0, 0, 0),  #
                 (1.75, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.75, 1.25, 6, 0, 0, 0),
                 (3.5, 2.5, 1.25, 4, 0, 0, 0)]

    # D3
    router_D3 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.3, 1.25, 3, 0, 0, 0),
                 (3.5, -0.3, 1.25, 14, 0, 0, 0),
                 (3.5, 1.75, 1.25, 8, 0, 0, 0),  #
                 (3.5, 2.5, 1.25, 3, 0, 0, 0)]

    # D2
    router_D2 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.3, 1.25, 3, 0, 0, 0),
                 (3.5, -0.3, 1.25, 14, 0, 0, 0),
                 (3.5, 1.25, 1.25, 7, 0, 0, 0),  #
                 (3.5, 2.5, 1.25, 6, 0, 0, 0)]

    # D1
    router_D1 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 1.25, 4, 0, 0, 0),
                 (0.75, -0.3, 1.25, 3, 0, 0, 0),
                 (3.5, -0.3, 1.25, 14, 0, 0, 0),
                 (3.5, 0.75, 1.25, 5, 0, 0, 0),  #
                 (3.5, 2.5, 1.25, 8, 0, 0, 0)]

    # D4
    router_D4 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (3.5, -0.3, 0.85, 14, 0, 0, 0),
                 (3.5, 0.75, 0.85, 5, 0, 0, 0),  #
                 (3.5, 2.5, 0.85, 8, 0, 0, 0)]

    # D5
    router_D5 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (3.5, -0.3, 0.85, 14, 0, 0, 0),
                 (3.5, 1.25, 0.85, 7, 0, 0, 0),  #
                 (3.5, 2.5, 0.85, 7, 0, 0, 0)]

    # D6
    router_D6 = [('x', 'y', 'z', 't', 'a', 'd', 's'),
                 (0, 0, 0.85, 3, 0, 0, 0),
                 (0.75, -0.3, 0.85, 3, 0, 0, 0),
                 (3.5, -0.3, 0.85, 14, 0, 0, 0),
                 (3.5, 1.75, 0.85, 10, 0, 0, 0),  #
                 (3.5, 2.5, 0.85, 3, 0, 0, 0)]

    dict_router = {'A1': router_A1, 'A2': router_A2, 'A3': router_A3, 'A4': router_A4, 'A5': router_A5, 'A6': router_A6,
                   'B1': router_B1, 'B2': router_B2, 'B3': router_B3, 'B4': router_B4, 'B5': router_B5, 'B6': router_B6,
                   'C1': router_C1, 'C2': router_C2, 'C3': router_C3, 'C4': router_C4, 'C5': router_C5, 'C6': router_C6,
                   'D1': router_D1, 'D2': router_D2, 'D3': router_D3, 'D4': router_D4, 'D5': router_D5, 'D6': router_D6}

    dict_road_break_index = {'A1': 3, 'A2': 3, 'A3': 3, 'A4': 3, 'A5': 3, 'A6': 3,
                             'B1': 5, 'B2': 5, 'B3': 5, 'B4': 5, 'B5': 5, 'B6': 5,
                             'C1': 5, 'C2': 5, 'C3': 5, 'C4': 5, 'C5': 5, 'C6': 5,
                             'D1': 5, 'D2': 5, 'D3': 5, 'D4': 5, 'D5': 5, 'D6': 5}

    dict_turn_duoji = {'A1': False, 'A2': False, 'A3': False, 'A4': False, 'A5': False, 'A6': False,
                       'B1': True, 'B2': True, 'B3': True, 'B4': True, 'B5': True, 'B6': True,
                       'C1': False, 'C2': False, 'C3': False, 'C4': False, 'C5': False, 'C6': False,
                       'D1': True, 'D2': True, 'D3': True, 'D4': True, 'D5': True, 'D6': True}

    dict_turn_laser = {'A1': False, 'A2': False, 'A3': False, 'A4': False, 'A5': False, 'A6': False,
                       'B1': True, 'B2': True, 'B3': True, 'B4': True, 'B5': True, 'B6': True,
                       'C1': False, 'C2': False, 'C3': False, 'C4': False, 'C5': False, 'C6': False,
                       'D1': True, 'D2': True, 'D3': True, 'D4': True, 'D5': True, 'D6': True}


    pass
