#!/usr/bin/env python3
"""Lista de materiales con costos (Fase H7).

Cruza la lista exportada por KiCad (hardware/kicad/out/findbot_bom.csv, generada por
kicad/build.sh) con los precios de bom/prices.yaml y agrega lo que va fuera de la placa.
Salidas: bom/bom_costos.csv y bom/resumen.md. Falla si algún componente no tiene precio.
Uso: python3 hardware/bom/gen_bom.py
"""
import csv
import re
import sys
from pathlib import Path

import yaml

BOM_DIR = Path(__file__).resolve().parent
KICAD_BOM = BOM_DIR.parent / 'kicad' / 'out' / 'findbot_bom.csv'


def cop(usd, rate):
    return round(usd * rate, -2)


def main():
    prices = yaml.safe_load((BOM_DIR / 'prices.yaml').read_text(encoding='utf-8'))
    rate = float(prices['usd_to_cop'])
    if not KICAD_BOM.exists():
        print(f'Falta {KICAD_BOM}: correr hardware/kicad/build.sh')
        return 1
    rows, missing = [], []
    with KICAD_BOM.open(encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            refs, value, qty, dnp = r['Ref'], r['Valor'], int(r['Cant']), r['No montar'] == 'DNP'
            if refs.startswith('R') and re.fullmatch(r'\d+([.,]\d+)?[kM]?\d*', value):
                unit, desc = float(prices['resistor_usd']), f'Resistencia {value}Ω 1/4 W 1 %'
            elif value in prices['board_by_value']:
                entry = prices['board_by_value'][value]
                unit, desc = float(entry['usd']), entry['desc']
            else:
                missing.append(f'{refs} ({value})')
                continue
            fase = 'No montar' if dnp else ('Fase 2' if 'fase 2' in value.lower() else 'MVP')
            rows.append({'Grupo': 'Placa base', 'Ref': refs, 'Descripción': desc, 'Valor': value,
                         'Cant': qty, 'USD unit': unit, 'Fase': fase})
    if missing:
        print('ERROR: sin precio en bom/prices.yaml → ' + ', '.join(missing))
        return 1
    rows.append({'Grupo': 'Placa base', 'Ref': 'PCB', 'Descripción': 'Fabricación de la PCB (5 unidades)',
                 'Valor': '', 'Cant': 1, 'USD unit': float(prices['pcb_fabrication_usd']), 'Fase': 'MVP'})
    for item in prices['offboard']:
        rows.append({'Grupo': item['grupo'], 'Ref': '', 'Descripción': item['item'], 'Valor': '',
                     'Cant': int(item['qty']), 'USD unit': float(item['usd']), 'Fase': item['fase']})

    for r in rows:
        r['USD subtotal'] = round(r['Cant'] * r['USD unit'], 2)
        r['COP subtotal'] = cop(r['USD subtotal'], rate)
    fields = ['Grupo', 'Ref', 'Descripción', 'Valor', 'Cant', 'USD unit', 'USD subtotal', 'COP subtotal', 'Fase']
    with (BOM_DIR / 'bom_costos.csv').open('w', encoding='utf-8', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    def total(pred):
        return round(sum(r['USD subtotal'] for r in rows if pred(r)), 2)

    groups = []
    for g in dict.fromkeys(r['Grupo'] for r in rows):
        groups.append((g, total(lambda r, g=g: r['Grupo'] == g and r['Fase'] == 'MVP'),
                       total(lambda r, g=g: r['Grupo'] == g and r['Fase'] == 'Fase 2')))
    mvp = total(lambda r: r['Fase'] == 'MVP')
    fase2 = total(lambda r: r['Fase'] == 'Fase 2')
    camera = total(lambda r: 'RealSense' in r['Descripción'])
    lines = [
        '# Costo estimado del robot findbot',
        '',
        f'> ⚠ Precios **estimados** ({prices["price_date"]}, minorista en línea, sin envío ni impuestos).',
        f'> Tasa de referencia: 1 USD = {rate:,.0f} COP. Actualizar `bom/prices.yaml` antes de comprar.',
        '> Generado por `hardware/bom/gen_bom.py`; el detalle por componente está en `bom_costos.csv`.',
        '',
        '| Grupo | MVP (USD) | Fase 2 (USD) |',
        '|---|---:|---:|',
    ]
    lines += [f'| {g} | {m:,.2f} | {f:,.2f} |' for g, m, f in groups]
    lines += [
        f'| **Total** | **{mvp:,.2f}** | **{fase2:,.2f}** |',
        '',
        f'- **MVP:** {mvp:,.2f} USD ≈ {cop(mvp, rate):,.0f} COP',
        f'- **Fase 2 (signos de vida):** {fase2:,.2f} USD ≈ {cop(fase2, rate):,.0f} COP',
        f'- **Robot completo:** {mvp + fase2:,.2f} USD ≈ {cop(mvp + fase2, rate):,.0f} COP',
        f'- La cámara RealSense D435 es el {100 * camera / (mvp + fase2):.0f} % del total; '
        'la tabla de sensores admite una Orbbec Astra como alternativa más económica.',
        '- Componentes "no montar" (pull-downs del caso B de los encoders) no suman.',
        '',
    ]
    (BOM_DIR / 'resumen.md').write_text('\n'.join(lines), encoding='utf-8')
    print(f'{len(rows)} líneas · MVP {mvp:,.2f} USD · Fase 2 {fase2:,.2f} USD · total {mvp + fase2:,.2f} USD')
    return 0


if __name__ == '__main__':
    sys.exit(main())
