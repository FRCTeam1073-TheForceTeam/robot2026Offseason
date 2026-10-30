/**
 * Parses JaCoCo HTML coverage report and prints a meaningful summary.
 * Usage: groovy scripts/print-coverage-summary.groovy
 */

def reportFile = new File("build/reports/jacoco/test/html/index.html")
if (!reportFile.exists()) {
    println("Coverage report not found at ${reportFile.absolutePath}")
    System.exit(1)
}

try {
    def html = reportFile.text

    // Parse overall totals (including hardware)
    def footerStart = html.indexOf("<tfoot>")
    def footerEnd = html.indexOf("</tfoot>")
    if (footerStart < 0 || footerEnd < 0) System.exit(1)

    def footer = html.substring(footerStart, footerEnd)
    def allPercentages = []
    def pattern = ~/class="ctr2">(\d+)%/
    footer.findAll(pattern) { match -> allPercentages.add(match[1]) }

    // Parse per-package coverage - extract each table row
    def packages = [:]
    def tbodyStart = html.indexOf("<tbody>")
    def tbodyEnd = html.indexOf("</tbody>")
    if (tbodyStart >= 0 && tbodyEnd > tbodyStart) {
        def tbody = html.substring(tbodyStart, tbodyEnd)
        // Split by table rows
        def rows = tbody.split(/<\/tr>/)
        rows.each { row ->
            def nameMatch = row =~ /el_package">([^<]+)<\/a>/
            if (nameMatch) {
                def pkgName = nameMatch[0][1]
                // After the package name, look for ctr2 cells with percentages
                def afterLink = row.substring(row.indexOf("</a>"))
                def percentPattern = />(\d+)%</
                def percentMatches = []
                afterLink.findAll(percentPattern) { fullMatch, num ->
                    percentMatches.add(num)
                }

                if (percentMatches.size() >= 2) {
                    packages[pkgName] = [
                        instr: percentMatches[0],
                        branch: percentMatches[1]
                    ]
                } else if (percentMatches.size() == 1) {
                    packages[pkgName] = [instr: percentMatches[0], branch: 'n/a']
                }
            }
        }
    }

    println("\n" + "=".repeat(70))
    println("Test Coverage Summary")
    println("=".repeat(70))

    if (allPercentages.size() >= 2) {
        println("OVERALL (including hardware initialization code)")
        println("  Instruction Coverage: ${allPercentages[0]}%")
        println("  Branch Coverage:      ${allPercentages[1]}%")
    }

    if (!packages.isEmpty()) {
        println("\nCOVERAGE BY PACKAGE:")
        // Highlight testable packages
        def testableOrder = [
            'frc.robot.utilities',
            'frc.robot.commands',
            'frc.robot.commands.Autos',
            'frc.robot.subsystems'
        ]

        testableOrder.each { pkgName ->
            if (packages.containsKey(pkgName)) {
                def cov = packages[pkgName]
                def label = pkgName
                if (pkgName == 'frc.robot.utilities') label += " (pure logic) ✓"
                if (pkgName.startsWith('frc.robot.commands')) label += " (decision trees) ✓"
                if (pkgName == 'frc.robot.subsystems') label += " (hardware+logic)"
                def instr = cov.instr ?: "?"
                def branch = cov.branch ?: "?"
                println("  ${label.padRight(50)} ${instr}% instr | ${branch}% branch")
            }
        }
    }

    println("\nNOTE: Utilities and Commands packages contain testable unit logic.")
    println("      Subsystems include untestable hardware initialization (constructors,")
    println("      CANbus config, signal setup) which lowers the apparent coverage.")
    println("\nFull Report: build/reports/jacoco/test/html/index.html")
    println("=".repeat(70) + "\n")
} catch (Exception e) {
    println("Could not parse coverage report: ${e.message}")
    e.printStackTrace()
    System.exit(1)
}
