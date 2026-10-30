param([Parameter(Mandatory=$true)][string[]]$Path)
$ErrorActionPreference = 'Stop'
foreach ($file in $Path) {
    $rows = Import-Csv -LiteralPath $file
    foreach ($group in ($rows | Group-Object connection,segment,controller)) {
        $duration=0.0; $squared=0.0; $maximum=0.0; $rateSquared=0.0; $saturated=0.0; $compute=0.0; $previous=$null; $speedSum=0.0
        foreach ($row in $group.Group) {
            $dt=[double]::Parse($row.dt_s,[cultureinfo]::InvariantCulture)
            $e=[double]::Parse($row.cte_m,[cultureinfo]::InvariantCulture)
            $u=[double]::Parse($row.steering,[cultureinfo]::InvariantCulture)
            $duration += $dt; $squared += $e*$e*$dt; $maximum=[math]::Max($maximum,[math]::Abs($e))
            if ($null -ne $previous) { $rateSquared += ($u-$previous)*($u-$previous)/$dt }
            if ([math]::Abs($u) -ge 0.999) { $saturated += $dt }
            $compute += [double]::Parse($row.compute_us,[cultureinfo]::InvariantCulture)
            $speedSum += [double]::Parse($row.speed_mph,[cultureinfo]::InvariantCulture)*$dt
            $previous=$u
        }
        [pscustomobject]@{
            File=$file; Run=$group.Name; Duration_s=$duration
            RMS_CTE_m=[math]::Sqrt($squared/$duration); Max_CTE_m=$maximum
            RMS_Steering_Rate=[math]::Sqrt($rateSquared/$duration)
            Saturation_Percent=100*$saturated/$duration
            Mean_Speed_mph=$speedSum/$duration; Mean_Compute_us=$compute/$group.Count
        }
    }
}
