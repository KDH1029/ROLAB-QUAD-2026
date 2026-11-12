import openvino as ov, numpy as np, sys
core = ov.Core(); m = core.read_model(sys.argv[1])
out = {}
for op in m.get_ops():
    if op.get_type_name() == 'Convolution':
        name = op.get_friendly_name()          # __module.model.0.conv/aten::_convolution/Convolution
        mod = name.split('/')[0].replace('__module.', '')
        w = op.input_value(1).get_node()
        while w.get_type_name() != 'Constant': w = w.input_value(0).get_node()
        W = w.get_data().astype(np.float32)
        # bias: Convolution -> Add(const)
        b = None
        for t in op.output(0).get_target_inputs():
            a = t.get_node()
            if a.get_type_name() == 'Add':
                for i in range(2):
                    n = a.input_value(i).get_node()
                    if n is op: continue
                    while n.get_type_name() != 'Constant' and n.get_input_size(): n = n.input_value(0).get_node()
                    if n.get_type_name() == 'Constant': b = n.get_data().astype(np.float32).reshape(-1)
        out[mod + '.weight'] = W
        if b is not None: out[mod + '.bias'] = b
        print(mod, W.shape, None if b is None else b.shape)
np.savez(sys.argv[2], **out)
