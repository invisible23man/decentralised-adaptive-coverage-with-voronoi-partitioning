def generate_experiment_tag(filter_config):
    tag = 'f'
    for k, v in filter_config.items():
        abbreviated_key = ''.join(word[0] for word in k.split('_'))  # Takes the first letter of each word in the key
        tag += f'_{abbreviated_key}{v}'
    return tag