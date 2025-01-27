import re
import os
print("Diretório atual:", os.getcwd())

def ajustar_labirinto(arquivo_entrada, arquivo_saida):
    with open(arquivo_entrada, 'r') as f:
        conteudo = f.read()

    # Expressão regular para localizar os nós Wall com nome <= 30
    padrao = r'(Wall\s*{[^}]*?name\s*"wall\((\d+)\)"[^}]*?size\s*)([0-9.]+)\s*([0-9.]+)\s*([0-9.]+)'
    
    def modificar_tamanho(match):
        # Verifica se o nome é <= 30
        numero = int(match.group(2))
        if numero <= 30:
            tamanho_atual = [float(match.group(3)), float(match.group(4)), float(match.group(5))]
            novo_tamanho = [t * 2 if i == 1 else t for i, t in enumerate(tamanho_atual)]  # Duplicar somente o segundo valor (y)
            tamanho_formatado = ' '.join(f"{t:.2f}" for t in novo_tamanho)
            return f"{match.group(1)}{tamanho_formatado}"
        return match.group(0)

    # Substituir no conteúdo do arquivo
    novo_conteudo = re.sub(padrao, modificar_tamanho, conteudo, flags=re.DOTALL)

    # Salvar no arquivo de saída
    with open(arquivo_saida, 'w') as f:
        f.write(novo_conteudo)

# Substitua pelos caminhos do seu arquivo
ajustar_labirinto('labirinto.wbt', 'labirinto_ajustado.wbt')
